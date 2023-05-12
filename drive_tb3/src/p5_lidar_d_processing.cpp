#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <chrono>

enum class RobotState {
    MOVING_STRAIGHT,
    TURNING_LEFT,
    TURNING_RIGHT,
    OUT_OF_MAZE
};

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor()
        : Node("lidar_processor"),
          state_(RobotState::MOVING_STRAIGHT)
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot_a/scan", default_qos, std::bind(&LidarProcessor::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot_a/cmd_vel", default_qos);
        desired_distance_ = 0.95; // Desired distance from the right wall
        complete_turn=true;
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float right_obstacle = *std::min_element(msg->ranges.begin() + 260, msg->ranges.begin() + 280);
        float front_obstacle = *std::min_element(msg->ranges.begin() + 340, msg->ranges.begin() + 360);
        float left_obstacle= *std::min_element(msg->ranges.begin() + 80, msg->ranges.begin() + 100);
        RCLCPP_INFO(this->get_logger(), "Front: %f, Right: %f, Left: %f,", front_obstacle, right_obstacle, left_obstacle);

        geometry_msgs::msg::Twist command;

        if(left_obstacle == std::numeric_limits<float>::infinity() &&
           right_obstacle == std::numeric_limits<float>::infinity() &&
           front_obstacle == std::numeric_limits<float>::infinity()) {
            state_ = RobotState::OUT_OF_MAZE;
        }

        switch (state_)
        {
            case RobotState::MOVING_STRAIGHT:
                if (front_obstacle < 1.22) { // threshold for obstacle distance
                    if (left_obstacle < right_obstacle) {
                        state_ = RobotState::TURNING_RIGHT;
                    } else {
                        state_ = RobotState::TURNING_LEFT;
                    }
                }
                break;

            case RobotState::TURNING_LEFT:
                if (front_obstacle > 1.22) {
                    state_ = RobotState::MOVING_STRAIGHT;
                }
                break;

            case RobotState::TURNING_RIGHT:
                if (front_obstacle > 1.22) {
                    state_ = RobotState::MOVING_STRAIGHT;
                }
                break;

            case RobotState::OUT_OF_MAZE:
                command.linear.x = 0.0;
                command.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Out of maze, stopping");
                break;
        }

        switch (state_)
        {
            case RobotState::MOVING_STRAIGHT:
                command.linear.x = 0.3;
                RCLCPP_INFO(this->get_logger(), "Moving straight");
                break;

                        case RobotState::TURNING_LEFT:
                command.linear.x = 0.0;
                command.angular.z = 0.5;
                RCLCPP_INFO(this->get_logger(), "Turning left");
                break;

            case RobotState::TURNING_RIGHT:
                command.linear.x = 0.0;
                command.angular.z = -0.5;
                RCLCPP_INFO(this->get_logger(), "Turning right");
                break;

            case RobotState::OUT_OF_MAZE:
                command.linear.x = 0.0;
                command.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Out of maze, stopping");
                break;
        }

        // Publish the command.
        pub_->publish(command);
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    float desired_distance_;
    RobotState state_;
    bool complete_turn;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}

