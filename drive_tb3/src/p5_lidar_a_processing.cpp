// Divided whole ray into 3 portions and log the minimum distance in each portion
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
        // Log the total number of rays
        RCLCPP_INFO(this->get_logger(), "Total number of rays: %zu", msg->ranges.size());

        // Divide the scan into three equal portions
        size_t size_per_portion = msg->ranges.size() / 3;
        auto range_begin = msg->ranges.begin();
        for (int i = 0; i < 3; ++i)
        {
            auto range_end = range_begin + size_per_portion;
            auto min_it = std::min_element(range_begin, range_end);
            if (min_it != range_end)
            {
                // Log the minimum distance (closest obstacle) in this portion
                RCLCPP_INFO(this->get_logger(), "Closest obstacle in portion %d: %f", i+1, *min_it);
            }
            range_begin = range_end;
        }
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
