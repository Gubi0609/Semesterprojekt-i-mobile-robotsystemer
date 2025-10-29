#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
    }

  private:
  void publish_vel(){
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";

    if (provider_) {
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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  geometry_msgs::msg::TwistStamped msg;

  std::shared_ptr<VelocityProvider> provider_;
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
