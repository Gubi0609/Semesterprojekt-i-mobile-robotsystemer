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
      if(provider_->getPreFunc() == 0 && provider_->state() == VelocityProvider::State::IDLE){
        provider_->updatePreFunc(1);
        provider_-> forwardForDuration(10.0, 50);
      }
      if(provider_->getPreFunc() ==1 && provider_->state() == VelocityProvider::State::IDLE){
        provider_ ->updatePreFunc(2);
        provider_-> turnForDuration(5.0, 50);
      }
      if(provider_->getPreFunc() ==2 && provider_->state() == VelocityProvider::State::IDLE){
        provider_ ->updatePreFunc(3);
        provider_-> turnForDuration(5.0, -50);
      }

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

  
      AudioComm::ChordReceiver receiver;
      AudioComm::ChordReceiver::Config recvConfig;
      recvConfig.fftSize = 16384;
      recvConfig.detectionTolerance = 150.0;
      recvConfig.minDetections = 2;       // CHANGED: Use safe mode (was 1)
      recvConfig.consistencyWindow = 0.3; // CHANGED: 0.3s window (was 0.1)
      recvConfig.updateRate = 75;         // CHANGED: 75Hz update rate (was 100)

      //"lightweight duplicate-detection state local to this thread"
      uint16_t lastValue = 0;
      std::chrono::steady_clock::time_point lastTimestamp = std::chrono::steady_clock::now();
      std::chrono::steady_clock::time_point lastActivityTime = std::chrono::steady_clock::now();
      const double LOCKOUT_PERIOD = 0.8;
      const double RESTART_INTERVAL = 30.0;  // Restart mic if idle for 30 seconds

      RCLCPP_INFO(this->get_logger(), "Starting audio receiver thread...");
      RCLCPP_INFO(this->get_logger(), "FFT Size: %d, Tolerance: %.1f Hz, MinDetections: %d",
      						recvConfig.fftSize, recvConfig.detectionTolerance, recvConfig.minDetections);
      RCLCPP_INFO(this->get_logger(), "Consistency Window: %.2fs, Update Rate: %.1f Hz",
      						recvConfig.consistencyWindow, recvConfig.updateRate);

       auto detectionCallback = [&](const AudioComm::ChordReceiver::Detection& det){
      RCLCPP_INFO(this->get_logger(), "🎵 RAW AUDIO DETECTED! Value: 0x%04X", det.value);

      auto now = std::chrono::steady_clock::now();

      // Update activity time on every detection
      lastActivityTime = now;

      if(lastValue == det.value){
        double elapsed = std::chrono::duration<double>(now - lastTimestamp).count();
        if(elapsed < LOCKOUT_PERIOD) { //ignore duplicate
      	RCLCPP_INFO(this->get_logger(), "🔄 DUPLICATE ignored (%.2fs ago)", elapsed);
      	return;
        }
      }
      lastValue = det.value;
      lastTimestamp = now;
        
        if(!crc.verify(det.value)){
          RCLCPP_WARN(rclcpp::get_logger("rb3_protocol"), "CRC failed for detection value 0x%04X", det.value);
          return;
        } else {
          RCLCPP_INFO(rclcpp::get_logger("rb3_protocol"), "✅ CRC PASSED for 0x%04X", det.value);
        }

        auto decoded = crc.decode1612(det.value);
        if(!decoded.has_value()){
          RCLCPP_WARN(rclcpp::get_logger("rb3_protocol"), "Decode failed for 0x%04X", det.value);
          return;
        }
        uint16_t command = decoded.value();
        RCLCPP_INFO(rclcpp::get_logger("rb3_protocol"), "🔓 DECODED command: 0x%03X", command);
        protocol.processCommand(command);

      };

      bool started = receiver.startReceiving(recvConfig, detectionCallback);
      if(!started){
        RCLCPP_ERROR(this->get_logger(), "❌ Audio receiver failed to start");
        receiver_running_.store(false);
        return;
      } else {
        RCLCPP_INFO(this->get_logger(), "✅ Audio receiver started successfully!");
        RCLCPP_INFO(this->get_logger(), "🎤 Listening for audio commands...");
      }

  		//run until asked to stop with periodic microphone restart
  		while(receiver_running_.load()){
  			std::this_thread::sleep_for(std::chrono::milliseconds(100));

  			// Check if we need to restart the microphone (when idle)
  			auto now = std::chrono::steady_clock::now();
  			auto timeSinceActivity = std::chrono::duration<double>(now - lastActivityTime).count();

  			if (timeSinceActivity >= RESTART_INTERVAL) {
  				RCLCPP_INFO(this->get_logger(), "⟳ Idle for %.0fs - Restarting microphone...", RESTART_INTERVAL);

  				// Stop receiver
  				receiver.stop();
  				std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Brief pause

  				// Restart receiver with same config and callback
  				started = receiver.startReceiving(recvConfig, detectionCallback);

  				if (started) {
  					RCLCPP_INFO(this->get_logger(), "✓ Microphone restarted successfully");
  				} else {
  					RCLCPP_ERROR(this->get_logger(), "✗ ERROR: Failed to restart microphone!");
  				}

  				// Reset activity timer
  				lastActivityTime = std::chrono::steady_clock::now();
  			}
  		}

  		//cleanup
  		receiver.stop();
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
