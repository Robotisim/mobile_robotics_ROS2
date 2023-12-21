#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pigpiod_if2.h>

const int PWM_RIGHT = 13;
const int MOTOR_RIGHT_FWD = 5;
const int MOTOR_RIGHT_REV = 11;

const int PWM_LEFT = 19;
const int MOTOR_LEFT_FWD = 26;
const int MOTOR_LEFT_REV = 6;

const double WHEEL_DIAMETER = 0.067; // meters
const double WHEELBASE = 0.185; // meters

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() : Node("motor_control")
    {
        pi = pigpio_setup();
        if (pi < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Pigpio Daemon. Is it running?");
            rclcpp::shutdown();
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MotorControlNode::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {

        double linear_velocity = msg->linear.x; // m/s
        double angular_velocity = msg->angular.z; // rad/s

        double right_velocity = (2 * linear_velocity + angular_velocity * WHEELBASE) / (2 * WHEEL_DIAMETER);
        double left_velocity = (2 * linear_velocity - angular_velocity * WHEELBASE) / (2 * WHEEL_DIAMETER);

        // Convert velocities to PWM values (you may need to adjust this)
        int right_pwm = static_cast<int>(right_velocity * 100);
        int left_pwm = static_cast<int>(left_velocity * 100);
        // RCLCPP_INFO(this->get_logger(), "Received cmd_vel - Linear Velocity: [X: %f, Y: %f, Z: %f],Angular Velocity: [Z: %f]", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
        // Drive motors
        RCLCPP_INFO(this->get_logger(), "PWM = %d / %d", left_pwm, right_pwm);

        drive_motors(right_pwm, left_pwm);
    }

    void drive_motors(int right_pwm, int left_pwm)
    {
        // Control right motor
        if (right_pwm > 0) {
            gpio_write(pi, MOTOR_RIGHT_FWD, 0);
            gpio_write(pi, MOTOR_RIGHT_REV, 1);
        } else {
            gpio_write(pi, MOTOR_RIGHT_FWD, 1);
            gpio_write(pi, MOTOR_RIGHT_REV, 0);
        }
        set_PWM_dutycycle(pi, PWM_RIGHT, std::abs(right_pwm));

        // Control left motor
        if (left_pwm > 0) {
            gpio_write(pi, MOTOR_LEFT_FWD, 0);
            gpio_write(pi, MOTOR_LEFT_REV, 1);
        } else {
            gpio_write(pi, MOTOR_LEFT_FWD, 1);
            gpio_write(pi, MOTOR_LEFT_REV, 0);
        }
        set_PWM_dutycycle(pi, PWM_LEFT, std::abs(left_pwm));
    }

    int pigpio_setup()
    {
        char *addrStr = NULL;
        char *portStr = NULL;
        const int pi = pigpio_start(addrStr, portStr);

        // Set up pins for right motor
        set_mode(pi, PWM_RIGHT, PI_OUTPUT);
        set_mode(pi, MOTOR_RIGHT_FWD, PI_OUTPUT);
        set_mode(pi, MOTOR_RIGHT_REV, PI_OUTPUT);

        // Set up pins for left motor
        set_mode(pi, PWM_LEFT, PI_OUTPUT);
        set_mode(pi, MOTOR_LEFT_FWD, PI_OUTPUT);
        set_mode(pi, MOTOR_LEFT_REV, PI_OUTPUT);

        // Initialize motors off
        gpio_write(pi, MOTOR_RIGHT_FWD, 1);
        gpio_write(pi, MOTOR_RIGHT_REV, 1);
        gpio_write(pi, MOTOR_LEFT_FWD, 1);
        gpio_write(pi, MOTOR_LEFT_REV, 1);

        return pi;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    int pi;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
