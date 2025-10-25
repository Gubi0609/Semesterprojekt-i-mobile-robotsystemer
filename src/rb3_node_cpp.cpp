#include <functional>
#include <memory>
#include <string>
#include <chrono>      // For std::chrono::seconds
#include <stdlib.h>    // For std::rand

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std;

class RB3_cpp_publisher : public rclcpp::Node{
  public:
    //Create publisher for publishing veloctiy commands to topic "cmd_vel"
    RB3_cpp_publisher()
    : Node("rb3_cpp_publisher"){
      RCLCPP_INFO(this->get_logger(), "Node started successfully!");
      publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel",10);

      // Timer to periodically publish data
      timer_ = this->create_wall_timer(1000ms, std::bind(&RB3_cpp_publisher::publish_vel, this));
    }

  private:
  //Class function to be called when you want to publish velocity commands
  void publish_vel(){
    // Populate the header
    msg.header.stamp = this->get_clock()->now(); // Set to current time		// CHECK om node og this kunne udskiftes
    msg.header.frame_id = "base_link";           // Reference frame

    // Populate the Twist part
    // Set angular velocity to desired value (ie. turning)
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = this->rand_FloatRange(0,2.84);

    // Set linear velocity to desired value (ie. forward and backwards)
    msg.twist.linear.x = this->rand_FloatRange(0,0.22);
    msg.twist.linear.y = 0;
    msg.twist.linear.z = 0;
    RCLCPP_INFO(this->get_logger(), "Publishing: %f , %f", this->msg.twist.linear.x, this->msg.twist.angular.z);
    publisher_->publish(msg);

  }

  // Make random numner between a and b
  float rand_FloatRange(float a, float b)
  {
    return ((b - a) * ((float)rand() / RAND_MAX)) + a;
  }

  // Private variables used for the publisher
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  geometry_msgs::msg::TwistStamped msg;
};
int main(int argc, char ** argv)
{
  // Initialize rclcpp
  rclcpp::init(argc, argv);
  // "Spin" the node
  rclcpp::spin(std::make_shared<RB3_cpp_publisher>());
  // Shutdown node when complete
  rclcpp::shutdown(); 
  return 0;
  
}
