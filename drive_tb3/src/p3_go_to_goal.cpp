#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

class GoToGoal : public rclcpp::Node
{
public:
    GoToGoal() : Node("go_to_goal")
    {
        this->declare_parameter<double>("goal_x", 2.0);
        this->declare_parameter<double>("goal_y", 3.0);
        this->declare_parameter<double>("kp", 5.0);
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&GoToGoal::odom_callback, this, std::placeholders::_1));

        goal_x_ = this->get_parameter("goal_x").as_double(); // Set your goal x
        goal_y_ = this->get_parameter("goal_y").as_double();  // Set your goal y
        Kp_ = this->get_parameter("kp").as_double();      // Set the proportional gain for control
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;

        // get the quaternion from the message
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        // Convert quaternion to RPY
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        float error_x = goal_x_ - x;
        float error_y = goal_y_ - y;
        float goal_distance = sqrt(pow(error_x, 2) + pow(error_y, 2));
        float goal_angle = atan2(error_y, error_x);
        float angle_error = goal_angle - yaw;
        geometry_msgs::msg::Twist vel_msg;

        // If not facing the goal
        if (abs(angle_error) > 0.1)
        {
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = Kp_ * (angle_error);

            RCLCPP_INFO(this->get_logger(), "Current angle: %f, Goal angle: %f,Angle Error: %f", yaw, goal_angle,angle_error);
        }
        else  // If facing the goal
        {
                    // RCLCPP_INFO(this->get_logger(), "Angle Set , lets move Straight " );

            if(goal_distance > 0.1)  // If not reached the goal
            {
                vel_msg.linear.x = Kp_ * goal_distance;
                vel_msg.angular.z = 0.0;

                RCLCPP_INFO(this->get_logger(), "Moving towards the goal. Current position: (%f, %f), Goal position: (%f, %f)", x, y, goal_x_, goal_y_);
            }
            else
            {
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;

                RCLCPP_INFO(this->get_logger(), "Reached the goal!");
            }
        }

        publisher_->publish(vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    float goal_x_;
    float goal_y_;
    float Kp_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToGoal>());
    rclcpp::shutdown();
    return 0;
}


