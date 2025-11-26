#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <stdlib.h>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "../LIB/audio_receiver.h"
#include "../LIB/frequency_detector.h"
#include "../LIB/audio_comm.h"
#include "../INCLUDE/CRC.h"
#include "../INCLUDE/command_protocol.h"

#include "velocityProvider.hpp"   // <-- new
using namespace std;
using namespace std::chrono_literals; // for 1000ms

class RB3_cpp_publisher : public rclcpp::Node{
  public:
    // accept an injected provider (can be nullptr for fallback/random)
    RB3_cpp_publisher(std::shared_ptr<VelocityProvider> provider = nullptr)
    : Node("rb3_cpp_publisher"), provider_(provider) {
      RCLCPP_INFO(this->get_logger(), "Node started successfully!");
      publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel",10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&RB3_cpp_publisher::publish_vel, this));
    

    //start the protocol receiver thread if provider present
    if(provider_){
      startProtocolReceiver();
    }
  }

  //stop the thread cleanly
  ~RB3_cpp_publisher() override{
    receiver_running_.store(false);
    if (receiver_thread_.joinable()) receiver_thread_.join();
  }

  private:
  void publish_vel(){
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";

    if (provider_) {
      provider_->update();
      msg.twist.linear.x = provider_->getVel();
      msg.twist.angular.z = provider_->getRot();
    } else {

      //if Provider is not created, the robot will stop and give logger info
      msg.twist.angular.x = 0;
      msg.twist.angular.y = 0;
      msg.twist.angular.z = 0;

      msg.twist.linear.x = 0;
      msg.twist.linear.y = 0;
      msg.twist.linear.z = 0;

      //added debugging
      RCLCPP_DEBUG(this->get_logger(), "publish_vel debug: provider not found. Publishing zero velocities.");

    }

    RCLCPP_INFO(this->get_logger(), "Publishing: %f , %f", this->msg.twist.linear.x, this->msg.twist.angular.z);
    publisher_->publish(msg);
  }

   void startProtocolReceiver(){
  //avoid starting multiple threads
  if(receiver_running_.load()) return;
  //writes atomic flag to true (threadsafe, and use the flag to tell the receiver thread to stop lateron)
  receiver_running_.store(true);

  //copy the ptr to provider (not the same as provider in main)
  std::shared_ptr<VelocityProvider> provider = provider_;

  receiver_thread_ = std::thread([this, provider](){
    CRC crc;
    CommandProtocol protocol;

    // Performance monitoring variables
    std::atomic<int> totalFFTs{0};          // Count all FFT operations
    std::atomic<int> totalDetections{0};     // Count chord detections
    std::atomic<int> validChords{0};         // Count valid CRC chords
    std::atomic<int> peaksDetected{0};       // Count total peaks
    std::atomic<int> validFreqs{0};          // Count valid frequency peaks
    auto perfStartTime = std::chrono::steady_clock::now();
    auto lastPerfReport = std::chrono::steady_clock::now();

      //map protocol callbacks -> provider actions
      protocol.setDriveForDurationCallback([provider](const DriveForDurationCommand& cmd){
        if(!provider) return;
        double d = cmd.getDurationSeconds();
        float speed = cmd.getSpeedPercent();
        //drive forward for duration (speed expected in percent 0..100)
        provider->driveForDuration(static_cast<float>(d), speed, 0.0f);
      });

      protocol.setTurnForDurationCallback([provider](const TurnForDurationCommand& cmd){
        if(!provider) return;
        double d = cmd.getDurationSeconds();
        float turn = cmd.getTurnRatePercent();
        provider->turnForDuration(static_cast<float>(d), turn);
     });
      
      protocol.setStopCallback([provider](){
        if(!provider) return;
        provider->setVel(0.0f);
        provider->setRot(0.0f);
        provider->setState(VelocityProvider::State::IDLE);
      });

      protocol.setDriveForwardCallback([provider](const DriveForwardCommand& cmd){
        if(!provider) return;
        float speed = cmd.getSpeedPercent();
        // Set continuous forward drive at specified speed (0-100 range)
        provider->setVel(speed);  // VelocityProvider expects 0-100 range directly
        provider->setRot(0.0f);   // No rotation for forward drive
        provider->setState(VelocityProvider::State::IDLE);  // Continuous mode uses IDLE state
      });

      protocol.setTurnCallback([provider](const TurnCommand& cmd){
        if(!provider) return;
        float turnRate = cmd.getTurnRatePercent();
        // Set continuous turn at specified rate (-100 to +100 range)
        provider->setVel(0.0f);     // No forward movement for pure turn
        provider->setRot(turnRate); // VelocityProvider expects -100 to +100 range directly
        provider->setState(VelocityProvider::State::IDLE);  // Continuous mode uses IDLE state
      });

      protocol.setModeChangeCallback([this](RobotMode mode){
        RCLCPP_INFO(this->get_logger(), "🔄 Mode changed to: %s", 
                    CommandProtocol::modeToString(mode).c_str());
        // You can add additional mode-specific logic here if needed
      });

  // Create decoder for chord analysis
AudioComm::ChordConfig chordConfig;
chordConfig.detectionTolerance = 150.0;
auto decoder = std::make_shared<AudioComm::ChordDecoder>(chordConfig);

// Create low-level frequency detector to track all FFT operations
FrequencyDetector freqDetector;
FrequencyDetector::Config detConfig;

// FAST MODE settings (Mode 2): 4096 FFT @ 20Hz - optimized for Pi
detConfig.sampleRate = 48000;
detConfig.fftSize = 4096;           // CHANGED: 4096 for speed (was 16384)
detConfig.numPeaks = 10;            // Look for up to 10 peaks
detConfig.duration = 0.0;           // Continuous
detConfig.bandpassLow = 0.0;        // DISABLED: No bandpass filtering (was 4000.0)
detConfig.bandpassHigh = 0.0;       // DISABLED: No bandpass filtering (was 17000.0)
detConfig.updateRate = 20.0;        // 20 Hz target update rate

// Consistency checking for chord detection
const int minDetections = 2;
const double consistencyWindow = 0.3;

      //"lightweight duplicate-detection state local to this thread"
      uint16_t lastValue = 0;
      std::chrono::steady_clock::time_point lastTimestamp = std::chrono::steady_clock::now();
      std::chrono::steady_clock::time_point lastActivityTime = std::chrono::steady_clock::now();
      const double LOCKOUT_PERIOD = 0.8;
      const double RESTART_INTERVAL = 30.0;  // Restart mic if idle for 30 seconds

      // Calculate frequency resolution
      double freqResolution = 48000.0 / detConfig.fftSize;

      RCLCPP_INFO(this->get_logger(), "Starting audio receiver thread...");
      RCLCPP_INFO(this->get_logger(), "Performance Mode: FAST (4096 FFT @ 20Hz)");
      RCLCPP_INFO(this->get_logger(), "FFT Size: %d, Frequency Resolution: %.2f Hz/bin",
      						detConfig.fftSize, freqResolution);
      RCLCPP_INFO(this->get_logger(), "Bandpass Filter: DISABLED (full spectrum)");
      RCLCPP_INFO(this->get_logger(), "Detection Tolerance: %.1f Hz, MinDetections: %d",
      						chordConfig.detectionTolerance, minDetections);
      RCLCPP_INFO(this->get_logger(), "Consistency Window: %.2fs, Target Update Rate: %.1f Hz",
      						consistencyWindow, detConfig.updateRate);
      RCLCPP_INFO(this->get_logger(), "FFT-level performance logging enabled (reports every 30s)");

      // Chord candidate tracking
      struct ChordCandidate {
      	int32_t value;
      	int count;
      	std::chrono::steady_clock::time_point firstSeen;
      	std::chrono::steady_clock::time_point lastSeen;
      };
      std::map<int32_t, ChordCandidate> chordCandidates;

      // Low-level FFT callback - called for EVERY FFT operation
      auto fftCallback = [&](const std::vector<FrequencyDetector::FrequencyPeak>& peaks){
      	totalFFTs++;  // Count every FFT operation
      	peaksDetected += peaks.size();

      	auto now = std::chrono::steady_clock::now();

      	// Analyze peaks to find tones
      	std::map<int, int> tonesFound;

      	for (const auto& peak : peaks) {
      		if (!decoder->isValidFrequency(peak.frequency)) {
      			continue;
      		}

      		validFreqs++;

      		auto [toneIndex, toneValue] = decoder->decodeSingleFrequency(peak.frequency);
      		if (toneIndex >= 0 && toneValue >= 0) {
      			if (tonesFound.find(toneIndex) == tonesFound.end()) {
      				tonesFound[toneIndex] = toneValue;
      			}
      		}
      	}

      	// Check if we have all 4 tones (complete chord)
      	if (tonesFound.size() == 4) {
      		// Build frequency vector
      		std::vector<double> detectedFreqs(4);
      		for (int i = 0; i < 4; ++i) {
      			int toneValue = tonesFound[i];
      			double minFreq, maxFreq;
      			switch(i) {
      				case 0: minFreq = 4500.0; maxFreq = 7000.0; break;
      				case 1: minFreq = 7500.0; maxFreq = 10000.0; break;
      				case 2: minFreq = 10500.0; maxFreq = 13000.0; break;
      				case 3: minFreq = 13500.0; maxFreq = 16000.0; break;
      				default: minFreq = maxFreq = 0;
      			}
      			double freqStep = (maxFreq - minFreq) / 15.0;
      			detectedFreqs[i] = minFreq + (toneValue * freqStep);
      		}

      		int32_t decodedValue = decoder->decodeFrequencies(detectedFreqs);

      		if (decodedValue >= 0) {
      			// Clean up old candidates
      			auto it = chordCandidates.begin();
      			while (it != chordCandidates.end()) {
      				double age = std::chrono::duration<double>(now - it->second.firstSeen).count();
      				if (age > consistencyWindow) {
      					it = chordCandidates.erase(it);
      				} else {
      					++it;
      				}
      			}

      			// Update or create candidate
      			if (chordCandidates.find(decodedValue) == chordCandidates.end()) {
      				chordCandidates[decodedValue] = {decodedValue, 1, now, now};
      			} else {
      				chordCandidates[decodedValue].count++;
      				chordCandidates[decodedValue].lastSeen = now;
      			}

      			// Check if confirmed
      			auto& candidate = chordCandidates[decodedValue];
      			if (candidate.count >= minDetections) {
      				double timeSinceLast = std::chrono::duration<double>(now - lastTimestamp).count();

      				if (decodedValue != lastValue || timeSinceLast > LOCKOUT_PERIOD) {
      					totalDetections++;
      					lastActivityTime = now;

      					uint16_t chordValue = static_cast<uint16_t>(decodedValue);
      					RCLCPP_INFO(this->get_logger(), "🎵 CHORD DETECTED! Value: 0x%04X", chordValue);

      					// Verify CRC
      					if(!crc.verify(chordValue)){
      						RCLCPP_WARN(rclcpp::get_logger("rb3_protocol"), "❌ CRC failed for 0x%04X", chordValue);
      						chordCandidates.erase(decodedValue);
      						lastValue = decodedValue;
      						lastTimestamp = now;
      						return;
      					}

      					RCLCPP_INFO(rclcpp::get_logger("rb3_protocol"), "✅ CRC PASSED for 0x%04X", chordValue);

      					// Decode command
      					auto decoded = crc.decode1612(chordValue);
      					if(!decoded.has_value()){
      						RCLCPP_WARN(rclcpp::get_logger("rb3_protocol"), "❌ Decode failed for 0x%04X", chordValue);
      						chordCandidates.erase(decodedValue);
      						lastValue = decodedValue;
      						lastTimestamp = now;
      						return;
      					}

      					uint16_t command = decoded.value();
      					validChords++;
      					RCLCPP_INFO(rclcpp::get_logger("rb3_protocol"), "🔓 DECODED command: 0x%03X", command);
      					protocol.processCommand(command);

      					lastValue = decodedValue;
      					lastTimestamp = now;
      					chordCandidates.erase(decodedValue);
      				}
      			}
      		}
      	}
      };

      bool started = freqDetector.startAsync(detConfig, fftCallback);
      if(!started){
      	RCLCPP_ERROR(this->get_logger(), "❌ Frequency detector failed to start");
      	receiver_running_.store(false);
      	return;
      } else {
      	RCLCPP_INFO(this->get_logger(), "✅ Frequency detector started successfully!");
      	RCLCPP_INFO(this->get_logger(), "🎤 Listening for audio commands...");
      }

  		//run until asked to stop with periodic microphone restart and performance reporting
  		while(receiver_running_.load()){
  			std::this_thread::sleep_for(std::chrono::milliseconds(100));

  			auto now = std::chrono::steady_clock::now();

  			// Performance reporting every 30 seconds
  			auto timeSinceLastReport = std::chrono::duration<double>(now - lastPerfReport).count();
  			if (timeSinceLastReport >= 30.0) {
  				auto totalRuntime = std::chrono::duration<double>(now - perfStartTime).count();
  				double fftRate = totalFFTs.load() / totalRuntime;
  				double detectionRate = totalDetections.load() / totalRuntime;
  				double validRate = validChords.load() / totalRuntime;
  				double avgPeaksPerFFT = totalFFTs.load() > 0 ?
  					(double)peaksDetected.load() / totalFFTs.load() : 0.0;

  				RCLCPP_INFO(this->get_logger(), "");
  				RCLCPP_INFO(this->get_logger(), "📊 ===== PERFORMANCE REPORT =====");
  				RCLCPP_INFO(this->get_logger(), "📊 Runtime: %.1f seconds", totalRuntime);
  				RCLCPP_INFO(this->get_logger(), "📊 FFT Processing:");
  				RCLCPP_INFO(this->get_logger(), "📊   Total FFTs: %d", totalFFTs.load());
  				RCLCPP_INFO(this->get_logger(), "📊   FFT Rate: %.2f Hz", fftRate);
  				RCLCPP_INFO(this->get_logger(), "📊   Total Peaks: %d", peaksDetected.load());
  				RCLCPP_INFO(this->get_logger(), "📊   Valid Freqs: %d", validFreqs.load());
  				RCLCPP_INFO(this->get_logger(), "📊   Avg Peaks/FFT: %.2f", avgPeaksPerFFT);
  				RCLCPP_INFO(this->get_logger(), "📊 Chord Detection:");
  				RCLCPP_INFO(this->get_logger(), "📊   Chord Detections: %d", totalDetections.load());
  				RCLCPP_INFO(this->get_logger(), "📊   Detection Rate: %.2f Hz", detectionRate);
  				RCLCPP_INFO(this->get_logger(), "📊   Valid Commands: %d", validChords.load());
  				RCLCPP_INFO(this->get_logger(), "📊   Valid Rate: %.2f Hz", validRate);

  				if (fftRate < 5.0) {
  					RCLCPP_WARN(this->get_logger(), "⚠️  FFT rate is very low (< 5 Hz) - CPU too slow or FFT too large");
  				} else if (fftRate < 15.0) {
  					RCLCPP_WARN(this->get_logger(), "⚠️  FFT rate is below target (< 15 Hz) - may cause delays");
  				} else if (fftRate >= 18.0) {
  					RCLCPP_INFO(this->get_logger(), "✅ FFT rate is excellent (>= 18 Hz)");
  				} else {
  					RCLCPP_INFO(this->get_logger(), "✓ FFT rate is good (15-18 Hz)");
  				}
  				RCLCPP_INFO(this->get_logger(), "📊 ==============================");
  				RCLCPP_INFO(this->get_logger(), "");

  				lastPerfReport = now;
  			}

  			// Check if we need to restart the microphone (when idle)
  			auto timeSinceActivity = std::chrono::duration<double>(now - lastActivityTime).count();

  			if (timeSinceActivity >= RESTART_INTERVAL) {
  				RCLCPP_INFO(this->get_logger(), "⟳ Idle for %.0fs - Restarting microphone...", RESTART_INTERVAL);

  				// Stop detector
  				freqDetector.stop();
  				std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Brief pause

  				// Restart detector with same config and callback
  				started = freqDetector.startAsync(detConfig, fftCallback);

  				if (started) {
  					RCLCPP_INFO(this->get_logger(), "✓ Microphone restarted successfully");
  				} else {
  					RCLCPP_ERROR(this->get_logger(), "✗ ERROR: Failed to restart microphone!");
  				}

  				// Reset activity timer and performance counters
  				lastActivityTime = std::chrono::steady_clock::now();
  				perfStartTime = std::chrono::steady_clock::now();
  				lastPerfReport = std::chrono::steady_clock::now();
  				totalFFTs.store(0);
  				totalDetections.store(0);
  				validChords.store(0);
  				peaksDetected.store(0);
  				validFreqs.store(0);
  			}
  		}

  		//cleanup
  		freqDetector.stop();

  		// Final performance report
  		auto endTime = std::chrono::steady_clock::now();
  		auto totalRuntime = std::chrono::duration<double>(endTime - perfStartTime).count();
  		double avgFFTRate = totalFFTs.load() / totalRuntime;
  		double avgDetectionRate = totalDetections.load() / totalRuntime;
  		double avgValidRate = validChords.load() / totalRuntime;

  		RCLCPP_INFO(this->get_logger(), "");
  		RCLCPP_INFO(this->get_logger(), "📊 ===== FINAL PERFORMANCE REPORT =====");
  		RCLCPP_INFO(this->get_logger(), "📊 Total Runtime: %.1f seconds", totalRuntime);
  		RCLCPP_INFO(this->get_logger(), "📊 FFT Statistics:");
  		RCLCPP_INFO(this->get_logger(), "📊   Total FFTs: %d", totalFFTs.load());
  		RCLCPP_INFO(this->get_logger(), "📊   Average FFT Rate: %.2f Hz", avgFFTRate);
  		RCLCPP_INFO(this->get_logger(), "📊   Total Peaks: %d", peaksDetected.load());
  		RCLCPP_INFO(this->get_logger(), "📊   Valid Frequencies: %d", validFreqs.load());
  		RCLCPP_INFO(this->get_logger(), "📊 Chord Detection:");
  		RCLCPP_INFO(this->get_logger(), "📊   Total Chord Detections: %d", totalDetections.load());
  		RCLCPP_INFO(this->get_logger(), "📊   Average Detection Rate: %.2f Hz", avgDetectionRate);
  		RCLCPP_INFO(this->get_logger(), "📊   Total Valid Commands: %d", validChords.load());
  		RCLCPP_INFO(this->get_logger(), "📊   Average Valid Command Rate: %.2f Hz", avgValidRate);
  		RCLCPP_INFO(this->get_logger(), "📊 ======================================");
  		RCLCPP_INFO(this->get_logger(), "");
  		RCLCPP_INFO(this->get_logger(), "Audio receiver thread stopped");

  	});
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  geometry_msgs::msg::TwistStamped msg;

  std::shared_ptr<VelocityProvider> provider_;

  std::thread receiver_thread_;
  std::atomic<bool> receiver_running_{false};  
};
// ...existing code...
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // create the provider from other file and inject it
  auto provider = std::make_shared<VelocityProvider>(); // ensure OtherClass is visible here (include its header)
  rclcpp::spin(std::make_shared<RB3_cpp_publisher>(provider));

  rclcpp::shutdown();
  return 0;
}
