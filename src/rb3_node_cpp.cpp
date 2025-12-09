#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <stdlib.h>
#include <cstdlib>
#include <optional>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "../LIB/audio_receiver.h"
#include "../LIB/frequency_detector.h"
#include "../LIB/audio_comm.h"
#include "../LIB/tone_generator.h"
#include "../INCLUDE/CRC.h"
#include "../INCLUDE/command_protocol.h"
#include "../INCLUDE/Database.h"
#include "../INCLUDE/Database.h"
#include "../INCLUDE/Logger.h"

#include "velocityProvider.hpp"   // <-- new
using namespace std;
using namespace std::chrono_literals; // for 1000ms

// Helper function to get current timestamp in milliseconds
int64_t getCurrentTimestampMs() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    return ms;
}

class RB3_cpp_publisher : public rclcpp::Node{
  public:
    // accept an injected provider (can be nullptr for fallback/random)
    RB3_cpp_publisher(std::shared_ptr<VelocityProvider> provider = nullptr)
    : Node("rb3_cpp_publisher"), provider_(provider) {
      RCLCPP_INFO(this->get_logger(), "Node started successfully!");
      publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel",10);
      timer_ = this->create_wall_timer(100ms, std::bind(&RB3_cpp_publisher::publish_vel, this));  // CHANGED: 1000ms -> 100ms (10Hz)
    
      // Initialize database for Pi reception logging
      db_ = std::make_unique<Database>("turtlebot_communication.db");
      if (!db_->open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open database. Continuing without logging...");
        db_.reset();
      } else if (!db_->createTables()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create database tables. Continuing without logging...");
        db_.reset();
      } else {
        RCLCPP_INFO(this->get_logger(), "Database initialized: turtlebot_communication.db");
      }
    

  	//start the protocol receiver thread if provider present
  	if(provider_){
  		startProtocolReceiver();
  		startKeyboardListener();  // Start keyboard listener for manual feedback testing
  	}
  }

  //stop the thread cleanly
  ~RB3_cpp_publisher() override{
    receiver_running_.store(false);
    keyboard_running_.store(false);
    if (receiver_thread_.joinable()) receiver_thread_.join();
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
    
    // Dump database to TEST_RESULTS before shutdown
    if (db_) {
      auto now = std::chrono::system_clock::now();
      auto time_t_now = std::chrono::system_clock::to_time_t(now);
      std::tm* tm_now = std::localtime(&time_t_now);
      std::ostringstream logFileName;
      // Get home directory from environment variable for portability
      const char* homeDir = std::getenv("HOME");
      if (homeDir != nullptr) {
        logFileName << homeDir << "/code_ws/src/rb3_package_cpp/temp_repo/TEST_RESULTS/turtlebot_log_"
                    << std::put_time(tm_now, "%Y%m%d_%H%M%S")
                    << ".txt";
      } else {
        // Fallback to current directory if HOME is not set
        logFileName << "turtlebot_log_"
                    << std::put_time(tm_now, "%Y%m%d_%H%M%S")
                    << ".txt";
      }
      
      RCLCPP_INFO(this->get_logger(), "Dumping database to %s...", logFileName.str().c_str());
      if (db_->dumpToLog(logFileName.str())) {
        RCLCPP_INFO(this->get_logger(), "Database log saved successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save database log.");
      }
      db_->close();
    }
  }  private:
  void publish_vel(){
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";

    if (provider_) {
      // if(provider_->getPreFunc() == 0 && provider_->state() == VelocityProvider::State::IDLE){
      //   provider_->updatePreFunc(1);
      //   provider_-> forwardForDuration(10.0, 50);
      // }
      // if(provider_->getPreFunc() ==1 && provider_->state() == VelocityProvider::State::IDLE){
      //   provider_ ->updatePreFunc(2);
      //   provider_-> turnForDuration(5.0, 50);
      // }
      // if(provider_->getPreFunc() ==2 && provider_->state() == VelocityProvider::State::IDLE){
      //   provider_ ->updatePreFunc(3);
      //   provider_-> turnForDuration(5.0, -50);
      // }

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

    // RCLCPP_INFO(this->get_logger(), "Publishing: %f , %f", this->msg.twist.linear.x, this->msg.twist.angular.z);  // REMOVED: Too spammy
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
      protocol.setDriveForDurationCallback([this, provider](const DriveForDurationCommand& cmd){
      	if(!provider) return;
      	double d = cmd.getDurationSeconds();
      	float speed = cmd.getSpeedPercent();
      	RCLCPP_INFO(this->get_logger(), "DRIVE command: %.1fs at %.0f%% speed", d, speed);
      	//drive forward for duration (speed expected in percent 0..100)
      	provider->forwardForDuration(static_cast<float>(d), speed);
      });

      protocol.setTurnForDurationCallback([this, provider](const TurnForDurationCommand& cmd){
     if(!provider) return;
     double d = cmd.getDurationSeconds();
     float turn = cmd.getTurnRatePercent();
     RCLCPP_INFO(this->get_logger(), "TURN command: %.1fs at %.0f%% turn rate", d, turn);
     provider->turnForDuration(static_cast<float>(d), turn);
     });
      
      protocol.setStopCallback([this, provider](){
      	if(!provider) return;
      	RCLCPP_INFO(this->get_logger(), "STOP command received");
      	provider->setVel(0.0f);
      	provider->setRot(0.0f);
      	provider->setState(VelocityProvider::State::IDLE);
      });

    protocol.setDriveForwardCallback([this, provider](const DriveForwardCommand& cmd){
  			if(!provider) return;
  			float speed = cmd.getSpeedPercent();
  			RCLCPP_INFO(this->get_logger(), "DRIVE CONTINUOUS command: %.0f%% speed", speed);
  			provider->driveContinuous(speed);
  		});

  		protocol.setTurnCallback([this, provider](const TurnCommand& cmd){
  			if(!provider) return;
  			float turn = cmd.getTurnRatePercent();
  			RCLCPP_INFO(this->get_logger(), "TURN CONTINUOUS command: %.0f%% turn rate", turn);
  			provider->turnContinuous(turn);
  		});

  // Create decoder for chord analysis
AudioComm::ChordConfig chordConfig;
chordConfig.detectionTolerance = 50.0;  // CHANGED: Narrowed from 150 Hz to 50 Hz
auto decoder = std::make_shared<AudioComm::ChordDecoder>(chordConfig);

// Create tone generator for feedback sounds (18-20 kHz band)
auto feedbackToneGen = std::make_shared<ToneGenerator>();

// Feedback sound configurations (audible range, avoid protocol bands)
// Confirmation tones - using two simultaneous frequencies for better noise immunity
const double FEEDBACK_SUCCESS_FREQ1 = 2500.0;  // Success tone 1 (2.5 kHz)
const double FEEDBACK_SUCCESS_FREQ2 = 3500.0;  // Success tone 2 (3.5 kHz)
const double FEEDBACK_FAILURE_FREQ1 = 2000.0;  // Failure tone 1 (2.0 kHz)
const double FEEDBACK_FAILURE_FREQ2 = 3000.0;  // Failure tone 2 (3.0 kHz)
const double FEEDBACK_DURATION = 0.4;          // 400ms tone duration

// Helper function to play feedback sound with two tones
auto playFeedbackSound = [this](double freq1, double freq2) {
	RCLCPP_INFO(this->get_logger(), "Feedback: %.0f Hz + %.0f Hz tones for 400ms", freq1, freq2);
	// Play both tones simultaneously using two speaker-test processes
	std::string cmd1 = "timeout 0.4 speaker-test -t sine -f " + std::to_string((int)freq1) +
	                   " -c 2 >/dev/null 2>&1 &";
	std::string cmd2 = "timeout 0.4 speaker-test -t sine -f " + std::to_string((int)freq2) +
	                   " -c 2 >/dev/null 2>&1 &";
	system(cmd1.c_str());
	system(cmd2.c_str());
};

// Create low-level frequency detector to track all FFT operations
FrequencyDetector freqDetector;
FrequencyDetector::Config detConfig;

// FAST MODE settings (Mode 2): 4096 FFT @ 20Hz - optimized for Pi
detConfig.sampleRate = 48000;
detConfig.fftSize = 4096;           // CHANGED: 4096 for speed (was 16384)
detConfig.numPeaks = 10;            // Look for up to 10 peaks
detConfig.duration = 0.0;           // Continuous
detConfig.bandpassLow = 4000.0;     // Filter out frequencies below 4000 Hz
detConfig.bandpassHigh = 17000.0;   // Filter out frequencies above 17000 Hz (removes 19980 Hz noise)
detConfig.updateRate = 20.0;        // 20 Hz target update rate

// Consistency checking for chord detection
const int minDetections = 3;  // CHANGED: Increased from 2 to 3 detections
const double consistencyWindow = 0.3;

      //"lightweight duplicate-detection state local to this thread"
      uint16_t lastValue = 0;
      std::chrono::steady_clock::time_point lastTimestamp = std::chrono::steady_clock::now();
      std::chrono::steady_clock::time_point lastActivityTime = std::chrono::steady_clock::now();
      const double LOCKOUT_PERIOD = 1.0;  // 1 second lockout for duplicate commands
      const double RESTART_INTERVAL = 30.0;  // Restart mic if idle for 30 seconds

      // Structure to collect data for database logging
      struct ReceptionData {
        int64_t timestamp = 0;
        float tone1 = 0.0f, tone2 = 0.0f, tone3 = 0.0f, tone4 = 0.0f;
        uint16_t commandBitEncoded = 0;
        bool crcValid = false;
        uint16_t commandBitDecoded = 0;
        std::string command = "";
        float speed = 0.0f;
        float turnSpeed = 0.0f;
        float duration = 0.0f;
        int confirmationSent = 0;  // 1 = positive, 2 = negative, 0 = none
      };

      // Calculate frequency resolution
      double freqResolution = 48000.0 / detConfig.fftSize;

      RCLCPP_INFO(this->get_logger(), "Starting audio receiver thread...");
      RCLCPP_INFO(this->get_logger(), "Performance Mode: FAST (4096 FFT @ 20Hz)");
      RCLCPP_INFO(this->get_logger(), "FFT Size: %d, Frequency Resolution: %.2f Hz/bin",
      						detConfig.fftSize, freqResolution);
      RCLCPP_INFO(this->get_logger(), "Bandpass Filter: 4000-17000 Hz");
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
      					RCLCPP_INFO(this->get_logger(), "CHORD DETECTED! Value: 0x%04X", chordValue);

      					// Verify CRC
      					if(!crc.verify(chordValue)){
      						RCLCPP_WARN(rclcpp::get_logger("rb3_protocol"), "CRC failed for 0x%04X", chordValue);
      						
      						// Log rejected data to database
      						if (this->db_) {
      							ReceptionData rxData;
      							rxData.timestamp = getCurrentTimestampMs();
      							for (size_t i = 0; i < detectedFreqs.size() && i < 4; ++i) {
      								switch(i) {
      									case 0: rxData.tone1 = detectedFreqs[i]; break;
      									case 1: rxData.tone2 = detectedFreqs[i]; break;
      									case 2: rxData.tone3 = detectedFreqs[i]; break;
      									case 3: rxData.tone4 = detectedFreqs[i]; break;
      								}
      							}
      							rxData.commandBitEncoded = chordValue;
      							rxData.crcValid = false;
      							rxData.commandBitDecoded = 0;  // No valid decode
      							rxData.command = "CRC_FAILED";
      							rxData.confirmationSent = 2;  // Negative confirmation
      							
      							this->db_->insertReceived(rxData.timestamp, rxData.tone1, rxData.tone2, 
      								rxData.tone3, rxData.tone4, rxData.commandBitEncoded, rxData.crcValid,
      								rxData.commandBitDecoded, rxData.command, rxData.speed, rxData.turnSpeed,
      								rxData.duration, rxData.confirmationSent);
      						}
      						
      						// Play failure sound
      						std::thread([playFeedbackSound, FEEDBACK_FAILURE_FREQ1, FEEDBACK_FAILURE_FREQ2]() {
      							playFeedbackSound(FEEDBACK_FAILURE_FREQ1, FEEDBACK_FAILURE_FREQ2);
      						}).detach();
      						chordCandidates.erase(decodedValue);
      						// Don't update lastValue/lastTimestamp for failed CRC
      						return;
      					}

      					RCLCPP_INFO(rclcpp::get_logger("rb3_protocol"), "CRC PASSED for 0x%04X", chordValue);

      					// Decode command
      					auto decoded = crc.decode1612(chordValue);
      					if(!decoded.has_value()){
      						RCLCPP_WARN(rclcpp::get_logger("rb3_protocol"), "Decode failed for 0x%04X", chordValue);
      						
      						// Log rejected data to database
      						if (this->db_) {
      							ReceptionData rxData;
      							rxData.timestamp = getCurrentTimestampMs();
      							for (size_t i = 0; i < detectedFreqs.size() && i < 4; ++i) {
      								switch(i) {
      									case 0: rxData.tone1 = detectedFreqs[i]; break;
      									case 1: rxData.tone2 = detectedFreqs[i]; break;
      									case 2: rxData.tone3 = detectedFreqs[i]; break;
      									case 3: rxData.tone4 = detectedFreqs[i]; break;
      								}
      							}
      							rxData.commandBitEncoded = chordValue;
      							rxData.crcValid = true;  // CRC passed but decode failed
      							rxData.commandBitDecoded = 0;  // No valid decode
      							rxData.command = "DECODE_FAILED";
      							rxData.confirmationSent = 2;  // Negative confirmation
      							
      							this->db_->insertReceived(rxData.timestamp, rxData.tone1, rxData.tone2, 
      								rxData.tone3, rxData.tone4, rxData.commandBitEncoded, rxData.crcValid,
      								rxData.commandBitDecoded, rxData.command, rxData.speed, rxData.turnSpeed,
      								rxData.duration, rxData.confirmationSent);
      						}
      						
      						// Play failure sound
      						std::thread([playFeedbackSound, FEEDBACK_FAILURE_FREQ1, FEEDBACK_FAILURE_FREQ2]() {
      							playFeedbackSound(FEEDBACK_FAILURE_FREQ1, FEEDBACK_FAILURE_FREQ2);
      						}).detach();
      						chordCandidates.erase(decodedValue);
      						// Don't update lastValue/lastTimestamp for failed decode
      						return;
      					}

      					uint16_t command = decoded.value();
      					validChords++;
      					RCLCPP_INFO(rclcpp::get_logger("rb3_protocol"), "DECODED command: 0x%03X", command);

      					// Prepare data for database logging
      					ReceptionData rxData;
      					rxData.timestamp = getCurrentTimestampMs();
      					for (size_t i = 0; i < detectedFreqs.size() && i < 4; ++i) {
      						switch(i) {
      							case 0: rxData.tone1 = detectedFreqs[i]; break;
      							case 1: rxData.tone2 = detectedFreqs[i]; break;
      							case 2: rxData.tone3 = detectedFreqs[i]; break;
      							case 3: rxData.tone4 = detectedFreqs[i]; break;
      						}
      					}
      					rxData.commandBitEncoded = chordValue;
      					rxData.crcValid = true;
      					rxData.commandBitDecoded = command;
      					rxData.confirmationSent = 1;  // Positive confirmation

      					// Determine command type based on protocol state machine
      					std::string cmdType = "UNKNOWN";

      					if (command == RESET_SIGNAL) {
      						// This is always a RESET command
      						cmdType = "RESET";
      					} else if (protocol.isWaitingForModeSelect()) {
      						// After RESET, this command selects the mode (no parameters yet)
      						switch (static_cast<RobotMode>(command)) {
      							case RobotMode::DRIVE_FOR_DURATION:
      								cmdType = "MODE_DRIVE_FOR_DURATION";
      								break;
      							case RobotMode::TURN_FOR_DURATION:
      								cmdType = "MODE_TURN_FOR_DURATION";
      								break;
      							case RobotMode::DRIVE_FORWARD:
      								cmdType = "MODE_DRIVE_FORWARD";
      								break;
      							case RobotMode::TURN:
      								cmdType = "MODE_TURN";
      								break;
      							case RobotMode::STOP:
      								cmdType = "STOP";
      								break;
      							default:
      								cmdType = "MODE_UNKNOWN";
      								break;
      						}
      						// No speed/duration/turnSpeed for mode selection commands
      					} else {
      						// Mode already selected - this command contains parameters
      						RobotMode currentMode = protocol.getCurrentMode();
      						switch (currentMode) {
      							case RobotMode::DRIVE_FOR_DURATION: {
      								cmdType = "DRIVE_FOR_DURATION_PARAMS";
      								auto driveCmd = DriveForDurationCommand::decode(command);
      								rxData.speed = driveCmd.getSpeedPercent();
      								rxData.duration = driveCmd.getDurationSeconds();
      								break;
      							}
      							case RobotMode::TURN_FOR_DURATION: {
      								cmdType = "TURN_FOR_DURATION_PARAMS";
      								auto turnCmd = TurnForDurationCommand::decode(command);
      								rxData.turnSpeed = turnCmd.getTurnRatePercent();
      								rxData.duration = turnCmd.getDurationSeconds();
      								break;
      							}
      							case RobotMode::DRIVE_FORWARD: {
      								cmdType = "DRIVE_FORWARD_PARAMS";
      								auto driveCmd = DriveForwardCommand::decode(command);
      								rxData.speed = driveCmd.getSpeedPercent();
      								break;
      							}
      							case RobotMode::TURN: {
      								cmdType = "TURN_PARAMS";
      								auto turnCmd = TurnCommand::decode(command);
      								rxData.turnSpeed = turnCmd.getTurnRatePercent();
      								break;
      							}
      							default:
      								cmdType = "PARAMS_UNKNOWN";
      								break;
      						}
      					}
      					rxData.command = cmdType;

      					// Play success sound before processing command
      					std::thread([playFeedbackSound, FEEDBACK_SUCCESS_FREQ1, FEEDBACK_SUCCESS_FREQ2]() {
      						playFeedbackSound(FEEDBACK_SUCCESS_FREQ1, FEEDBACK_SUCCESS_FREQ2);
      					}).detach();

      					// Log successful reception to database
      					if (this->db_) {
      						this->db_->insertReceived(rxData.timestamp, rxData.tone1, rxData.tone2,
      							rxData.tone3, rxData.tone4, rxData.commandBitEncoded, rxData.crcValid,
      							rxData.commandBitDecoded, rxData.command, rxData.speed, rxData.turnSpeed,
      							rxData.duration, rxData.confirmationSent);
      					}

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
      	RCLCPP_ERROR(this->get_logger(), "Frequency detector failed to start");
      	receiver_running_.store(false);
      	return;
      } else {
      	RCLCPP_INFO(this->get_logger(), "Frequency detector started successfully!");
      	RCLCPP_INFO(this->get_logger(), "Listening for audio commands...");
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
  				RCLCPP_INFO(this->get_logger(), "===== PERFORMANCE REPORT =====");
  				RCLCPP_INFO(this->get_logger(), "Runtime: %.1f seconds", totalRuntime);
  				RCLCPP_INFO(this->get_logger(), "FFT Processing:");
  				RCLCPP_INFO(this->get_logger(), "  Total FFTs: %d", totalFFTs.load());
  				RCLCPP_INFO(this->get_logger(), "  FFT Rate: %.2f Hz", fftRate);
  				RCLCPP_INFO(this->get_logger(), "  Total Peaks: %d", peaksDetected.load());
  				RCLCPP_INFO(this->get_logger(), "  Valid Freqs: %d", validFreqs.load());
  				RCLCPP_INFO(this->get_logger(), "  Avg Peaks/FFT: %.2f", avgPeaksPerFFT);
  				RCLCPP_INFO(this->get_logger(), "Chord Detection:");
  				RCLCPP_INFO(this->get_logger(), "  Chord Detections: %d", totalDetections.load());
  				RCLCPP_INFO(this->get_logger(), "  Detection Rate: %.2f Hz", detectionRate);
  				RCLCPP_INFO(this->get_logger(), "  Valid Commands: %d", validChords.load());
  				RCLCPP_INFO(this->get_logger(), "  Valid Rate: %.2f Hz", validRate);

  				if (fftRate < 5.0) {
  					RCLCPP_WARN(this->get_logger(), " FFT rate is very low (< 5 Hz) - CPU too slow or FFT too large");
  				} else if (fftRate < 15.0) {
  					RCLCPP_WARN(this->get_logger(), " FFT rate is below target (< 15 Hz) - may cause delays");
  				} else if (fftRate >= 18.0) {
  					RCLCPP_INFO(this->get_logger(), " FFT rate is excellent (>= 18 Hz)");
  				} else {
  					RCLCPP_INFO(this->get_logger(), " FFT rate is good (15-18 Hz)");
  				}
  				RCLCPP_INFO(this->get_logger(), " ==============================");
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
  					RCLCPP_INFO(this->get_logger(), "Microphone restarted successfully");
  				} else {
  					RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to restart microphone!");
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
  		RCLCPP_INFO(this->get_logger(), "===== FINAL PERFORMANCE REPORT =====");
  		RCLCPP_INFO(this->get_logger(), "Total Runtime: %.1f seconds", totalRuntime);
  		RCLCPP_INFO(this->get_logger(), "FFT Statistics:");
  		RCLCPP_INFO(this->get_logger(), "  Total FFTs: %d", totalFFTs.load());
  		RCLCPP_INFO(this->get_logger(), "  Average FFT Rate: %.2f Hz", avgFFTRate);
  		RCLCPP_INFO(this->get_logger(), "  Total Peaks: %d", peaksDetected.load());
  		RCLCPP_INFO(this->get_logger(), "  Valid Frequencies: %d", validFreqs.load());
  		RCLCPP_INFO(this->get_logger(), "Chord Detection:");
  		RCLCPP_INFO(this->get_logger(), "  Total Chord Detections: %d", totalDetections.load());
  		RCLCPP_INFO(this->get_logger(), "  Average Detection Rate: %.2f Hz", avgDetectionRate);
  		RCLCPP_INFO(this->get_logger(), "  Total Valid Commands: %d", validChords.load());
  		RCLCPP_INFO(this->get_logger(), "  Average Valid Command Rate: %.2f Hz", avgValidRate);
  		RCLCPP_INFO(this->get_logger(), "======================================");
  		RCLCPP_INFO(this->get_logger(), "");
  		RCLCPP_INFO(this->get_logger(), "Audio receiver thread stopped");

		});
	}

	void startKeyboardListener() {
		if(keyboard_running_.load()) return;
		keyboard_running_.store(true);

		keyboard_thread_ = std::thread([this]() {
			// Set terminal to non-blocking mode
			struct termios oldt, newt;
			tcgetattr(STDIN_FILENO, &oldt);
			newt = oldt;
			newt.c_lflag &= ~(ICANON | ECHO);
			tcsetattr(STDIN_FILENO, TCSANOW, &newt);

			int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
			fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

			RCLCPP_INFO(this->get_logger(), "");
			RCLCPP_INFO(this->get_logger(), "Keyboard Controls:");
			RCLCPP_INFO(this->get_logger(), "  Press 's' - Play SUCCESS tone (3.5 kHz)");
			RCLCPP_INFO(this->get_logger(), "  Press 'f' - Play FAILURE tone (4.0 kHz)");
			RCLCPP_INFO(this->get_logger(), "  Press 'q' - Quit keyboard listener");
			RCLCPP_INFO(this->get_logger(), "");

			auto toneGen = std::make_shared<ToneGenerator>();
			const double SUCCESS_FREQ1 = 2500.0;
			const double SUCCESS_FREQ2 = 3500.0;
			const double FAILURE_FREQ1 = 2000.0;
			const double FAILURE_FREQ2 = 3000.0;

			auto playTwoTones = [this](double freq1, double freq2, const char* name) {
				RCLCPP_INFO(this->get_logger(), " Playing %s tones: %.0f Hz + %.0f Hz for 400ms", name, freq1, freq2);
				// Play both tones simultaneously
				std::string cmd1 = "timeout 0.4 speaker-test -t sine -f " + std::to_string((int)freq1) +
				                   " -c 2 >/dev/null 2>&1 &";
				std::string cmd2 = "timeout 0.4 speaker-test -t sine -f " + std::to_string((int)freq2) +
				                   " -c 2 >/dev/null 2>&1 &";
				system(cmd1.c_str());
				system(cmd2.c_str());
				RCLCPP_INFO(this->get_logger(), "Tone commands sent");
			};

			while(keyboard_running_.load()) {
				char c = getchar();
				if (c == 's' || c == 'S') {
					playTwoTones(SUCCESS_FREQ1, SUCCESS_FREQ2, "SUCCESS");
				} else if (c == 'f' || c == 'F') {
					playTwoTones(FAILURE_FREQ1, FAILURE_FREQ2, "FAILURE");
				} else if (c == 'q' || c == 'Q') {
					RCLCPP_INFO(this->get_logger(), "Keyboard listener stopped");
					keyboard_running_.store(false);
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}

			// Restore terminal settings
			tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
			fcntl(STDIN_FILENO, F_SETFL, oldf);
		});
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
	geometry_msgs::msg::TwistStamped msg;

	std::shared_ptr<VelocityProvider> provider_;
	std::unique_ptr<Database> db_;  // Database for logging received commands

	std::thread receiver_thread_;
	std::atomic<bool> receiver_running_{false};
	std::thread keyboard_thread_;
	std::atomic<bool> keyboard_running_{false};
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
