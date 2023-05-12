// Finding out pairs of rays for left,right,front,back
// 173-183 -> front
// 130-140 -> right
// 40 - 50 ->left
// 85-95 -> back

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <vector>

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

        // Divide the scan into pairs
        size_t pair_size = 2;
        size_t pairs_count = msg->ranges.size() / pair_size;

        std::vector<float> min_distances(pairs_count);

        // Pairs
        // 173-183 -> front
        // 130-140 -> right
        // 40 - 50 ->left
        // 85-95 -> back

        auto range_begin = msg->ranges.begin();
        for (size_t i = 0; i < pairs_count; ++i)
        {
            auto range_end = range_begin + pair_size;
            auto min_it = std::min_element(range_begin, range_end);

            if (min_it != range_end)
            {
                // Store the minimum distance (closest obstacle) in this pair
                min_distances[i] = *min_it;

                // Log the minimum distance (closest obstacle) in this pair
                RCLCPP_INFO(this->get_logger(), "Closest obstacle in pair %zu: %f", i+1, *min_it);
            }

            range_begin = range_end;
        }

        // Check the left, right, and front
        if (!min_distances.empty())
        {
            size_t mid = pairs_count / 2;
            float left_obstacle = min_distances[0]; // considering left-most pair for left side
            float right_obstacle = min_distances[pairs_count - 1]; // considering right-most pair for right side
            float front_obstacle = min_distances[mid]; // considering middle pair for front

            RCLCPP_INFO(this->get_logger(), "Left obstacle: %f, Front obstacle: %f, Right obstacle: %f", left_obstacle, front_obstacle, right_obstacle);
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
