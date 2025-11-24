#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <stdlib.h>

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
        float speed cmd.getSpeedPercent();
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
      AudioComm::chordReceiver::Config recvConfig;
      recvConfig.fftSize = 16384;
      recvConfig.detectionTolerance = 150.0;
      recvConfig.minDetections = 2;
      recvConfig.consistencyWindow = 0.3;
      recvConfig.updateRate = 75.0;

      //"lightweight duplicate-detection state local to this thread"
      uint16_t lastValue = 0;
      std::chrono::steady_clock::time_point lastTimestamp = std::chrono::steady_clock::now();
      const double LOCKOUT_PERIOD = 0.8;

      auto detectionCallback = [&crc, &protocol, &lastValue, &lastTimestamp, &receiver](const AudioComm::ChordReceiver::Detection& det){
        auto now = std::chrono::steady_clock::now();
        if(lastValue ==det.value){
          double elapsed = std::chrono::duration<double>(now - lastTimestamp).count();
          if(elapsed <LOCKOUT_PERIOD) return; //ignore duplicate
        }
        lastValue = det.value;
        lastTimestamp = now;
        
        if(!crc.verify(det.value)){
          RCLCPP_WARN(rclcpp::get_logger("rb3_protocol"), "CRC failed for detection value 0x%04X", det.value);
          return;
        }

        auto decoded = crc.decode1612(det.value);
        if(!decoded.hast_value()){
          RCLCPP_WARN(rclcpp::get_logger("rb3_protocol"), "Decode failed for 0x%04X", det.value);
          return;
        }
        uint16_t command = decoded.value();
        protocol.processCommand(command);

      };

      bool started = receiver.startReceiving(recvConfig, detectionCallback);
      if(!started){
        RCLCPP_ERROR(this->get_logger(), "protocol receiver failed to start");
        receiver_running_.store(false);
        return;
      }

      //run until asked to stop
      while(receiver_running_.load()){
        std::this_thread::sleep_for(100 ms);
      }

      //cleanup
      receiver.stop();

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
