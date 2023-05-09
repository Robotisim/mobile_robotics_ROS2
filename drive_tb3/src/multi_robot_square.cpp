#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MultiRobotPublisher : public rclcpp::Node {
public:
  MultiRobotPublisher() : Node("multi_robot_publisher") {
    // Declare parameters
    this->declare_parameter<std::string>("cmd_vel_topic_a", "/robot_a/cmd_vel");
    this->declare_parameter<std::string>("cmd_vel_topic_b", "/robot_b/cmd_vel");
    this->declare_parameter<std::string>("cmd_vel_topic_c", "/robot_c/cmd_vel");
    this->declare_parameter<double>("linear_velocity", 2.0);

    // Get parameter values
    cmd_vel_topic_a_ = this->get_parameter("cmd_vel_topic_a").as_string();
    cmd_vel_topic_b_ = this->get_parameter("cmd_vel_topic_b").as_string();
    cmd_vel_topic_c_ = this->get_parameter("cmd_vel_topic_c").as_string();
    linear_velocity_ = this->get_parameter("linear_velocity").as_double();

    // Create publishers
    cmd_vel_pub_a_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_a_, 10);
    cmd_vel_pub_b_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_b_, 10);
    cmd_vel_pub_c_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_c_, 10);

    // Create timer to publish messages periodically
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&MultiRobotPublisher::timer_callback, this));
  }

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_a_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_b_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_c_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string cmd_vel_topic_a_;
  std::string cmd_vel_topic_b_;
  std::string cmd_vel_topic_c_;
  double linear_velocity_;

  // Timer callback function
  void timer_callback() {
    // Create twist message
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_velocity_;
    twist.angular.z = 0.0;

    // Publish messages on all topics
    cmd_vel_pub_a_->publish(twist);
    cmd_vel_pub_b_->publish(twist);
    cmd_vel_pub_c_->publish(twist);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiRobotPublisher>());
  rclcpp::shutdown();
  return 0;
}
