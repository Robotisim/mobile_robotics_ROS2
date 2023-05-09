#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SquareDrive : public rclcpp::Node {
public:
  SquareDrive() : Node("square_drive") {
    // Declare parameters
    this->declare_parameter<std::string>("cmd_vel_topic", "/robot_a/cmd_vel");
    this->declare_parameter<double>("linear_velocity", 0.2);
    this->declare_parameter<double>("angular_velocity", 1.0);
    this->declare_parameter<double>("side_length", 0.5);

    // Get parameter values
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    linear_velocity_ = this->get_parameter("linear_velocity").as_double();
    angular_velocity_ = this->get_parameter("angular_velocity").as_double();
    side_length_ = this->get_parameter("side_length").as_double();

    // Create publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    // Create timer to drive the robot in a square pattern
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&SquareDrive::timer_callback, this));
  }

private:
  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string cmd_vel_topic_;
  double linear_velocity_;
  double angular_velocity_;
  double side_length_;

  // Timer callback function
  void timer_callback() {
    // Create twist message
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_velocity_;
    twist.angular.z = 0.0;

    // Drive forward for the specified side length
  double travel_time = side_length_ / linear_velocity_;
  cmd_vel_pub_->publish(twist);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(travel_time)));

  // Stop the robot and turn
  twist.linear.x = 0.0;
  twist.angular.z = angular_velocity_;
  double turn_time = M_PI / (2.0 * angular_velocity_);
  cmd_vel_pub_->publish(twist);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(turn_time)));

  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareDrive>());
  rclcpp::shutdown();
  return 0;
}
