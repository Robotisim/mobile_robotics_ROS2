#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtlesimStraightLine : public rclcpp::Node {
  public:
    TurtlesimStraightLine() : Node("robot_straight") {
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                       std::bind(&TurtlesimStraightLine::timer_callback, this));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback() {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 2.0;
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimStraightLine>());
  rclcpp::shutdown();
  return 0;
}
