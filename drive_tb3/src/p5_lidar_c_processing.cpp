#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor()
        : Node("lidar_processor")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot_a/scan", default_qos, std::bind(&LidarProcessor::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Ray pairs
        // 173-183 -> front
        // 130-140 -> right
        // 40 - 50 -> left
        // 85-95 -> back

        // Assume pairs are valid and within range -> as all values were pairs of 2
        float back_obstacle= *std::min_element(msg->ranges.begin() + 160, msg->ranges.begin() + 180);
        float right_obstacle = *std::min_element(msg->ranges.begin() + 260, msg->ranges.begin() + 280);
        float front_obstacle = *std::min_element(msg->ranges.begin() + 340, msg->ranges.begin() + 360);
        float left_obstacle= *std::min_element(msg->ranges.begin() + 80, msg->ranges.begin() + 100);

        // Log the minimum distance (closest obstacle) in each direction
        RCLCPP_INFO(this->get_logger(), "Front obstacle: %f", front_obstacle);
        RCLCPP_INFO(this->get_logger(), "Right obstacle: %f", right_obstacle);
        RCLCPP_INFO(this->get_logger(), "Left obstacle: %f", left_obstacle);
        RCLCPP_INFO(this->get_logger(), "Back obstacle: %f", back_obstacle);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}
