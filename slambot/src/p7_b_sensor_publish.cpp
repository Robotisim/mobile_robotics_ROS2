#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int16.hpp"
#include <pigpiod_if2.h>
#include <iostream>

// IMU Setup
const int MPU9250_ADDRESS = 0x68; // MPU9250 address
const int AK8963_ADDRESS = 0x0C; // AK8963 address
const int I2Cbus = 1; // RPi typical I2C bus

// Encoder Pins
const int ENCODER_RIGHT_A = 17;
const int ENCODER_RIGHT_B = 27;
const int ENCODER_LEFT_A = 20;
const int ENCODER_LEFT_B = 21;

int encoder_count_right = 0;
int encoder_count_left = 0;

class SensorNode : public rclcpp::Node
{
public:
    SensorNode() : Node("sensor_publisher_node")
    {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
        encoder_right_publisher_ = this->create_publisher<std_msgs::msg::Int16>("encoder_right", 10);
        encoder_left_publisher_ = this->create_publisher<std_msgs::msg::Int16>("encoder_left", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorNode::publish_data, this));
        pi = pigpio_start(NULL, NULL);
        MPU9250_HANDLE = i2c_open(pi, I2Cbus, MPU9250_ADDRESS, 0);
        setup_mpu9250();
        setup_encoders();
    }

    ~SensorNode()
    {
        i2c_close(pi, MPU9250_HANDLE);
        pigpio_stop(pi);
    }

private:
    void setup_mpu9250()
    {
        // Wake up MPU9250
        i2c_write_byte_data(pi, MPU9250_HANDLE, 0x6B, 0x00);
        time_sleep(0.1);

        // Enable bypass mode to access AK8963 directly
        i2c_write_byte_data(pi, MPU9250_HANDLE, 0x37, 0x02);
        time_sleep(0.1);

        // Open AK8963
        int AK8963_HANDLE = i2c_open(pi, I2Cbus, AK8963_ADDRESS, 0);
        if (AK8963_HANDLE < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open I2C comms with AK8963");
            return;
        }

        // Reset AK8963
        i2c_write_byte_data(pi, AK8963_HANDLE, 0x0B, 0x01);
        time_sleep(0.1);

        // Set AK8963 to continuous measurement mode
        i2c_write_byte_data(pi, AK8963_HANDLE, 0x0A, 0x16); // 16-bit output, continuous mode 2
        time_sleep(0.1);

        i2c_close(pi, AK8963_HANDLE);
    }

    void setup_encoders()
    {
        // Set up encoder pins as inputs with pull-up resistors
        set_mode(pi, ENCODER_RIGHT_A, PI_INPUT);
        set_mode(pi, ENCODER_RIGHT_B, PI_INPUT);
        set_mode(pi, ENCODER_LEFT_A, PI_INPUT);
        set_mode(pi, ENCODER_LEFT_B, PI_INPUT);
        set_pull_up_down(pi, ENCODER_RIGHT_A, PI_INPUT);
        set_pull_up_down(pi, ENCODER_RIGHT_B, PI_INPUT);
        set_pull_up_down(pi, ENCODER_LEFT_A, PI_INPUT);
        set_pull_up_down(pi, ENCODER_LEFT_B, PI_INPUT);

        // Set up callbacks for encoder interrupts
        callback(pi, ENCODER_RIGHT_A, EITHER_EDGE, encoder_callback_right);
        callback(pi, ENCODER_LEFT_A, EITHER_EDGE, encoder_callback_left);
    }

static void encoder_callback_right(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    if (gpio_read(pi, ENCODER_RIGHT_A) == gpio_read(pi, ENCODER_RIGHT_B)) {
        encoder_count_right--;
    } else {
        encoder_count_right++;
    }
}

static void encoder_callback_left(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    if (gpio_read(pi, ENCODER_LEFT_A) == gpio_read(pi, ENCODER_LEFT_B)) {
        encoder_count_left--;
    } else {
        encoder_count_left++;
    }
}


    void get_imu_data(sensor_msgs::msg::Imu &msg)
    {
        // Read accelerometer and gyroscope data
    int16_t ax = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3B) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3C));
    int16_t ay = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3D) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3E));
    int16_t az = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x3F) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x40));
    int16_t gx = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x43) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x44));
    int16_t gy = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x45) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x46));
    int16_t gz = (int16_t)(i2c_read_byte_data(pi, MPU9250_HANDLE, 0x47) << 8 | i2c_read_byte_data(pi, MPU9250_HANDLE, 0x48));

    // cout << "Accelerometer - X: " << ax << ", Y: " << ay << ", Z: " << az << endl;
    // cout << "Gyroscope - X: " << gx << ", Y: " << gy << ", Z: " << gz << endl;

    // Open AK8963
    int AK8963_HANDLE = i2c_open(pi, I2Cbus, AK8963_ADDRESS, 0);

    // Read magnetometer data
    int16_t mx = (int16_t)(i2c_read_byte_data(pi, AK8963_HANDLE, 0x03) << 8 | i2c_read_byte_data(pi, AK8963_HANDLE, 0x04));
    int16_t my = (int16_t)(i2c_read_byte_data(pi, AK8963_HANDLE, 0x05) << 8 | i2c_read_byte_data(pi, AK8963_HANDLE, 0x06));
    int16_t mz = (int16_t)(i2c_read_byte_data(pi, AK8963_HANDLE, 0x07) << 8 | i2c_read_byte_data(pi, AK8963_HANDLE, 0x08));

    // cout << "Magnetometer - X: " << mx << ", Y: " << my << ", Z: " << mz << endl;

    i2c_close(pi, AK8963_HANDLE);

        // Fill the IMU message
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;
    }

    void publish_data()
    {
        sensor_msgs::msg::Imu imu_msg;
        get_imu_data(imu_msg);
        imu_msg.header.frame_id="test_origin";
        imu_publisher_->publish(imu_msg);

        std_msgs::msg::Int16 msg_right;
        std_msgs::msg::Int16 msg_left;
        msg_right.data = encoder_count_right;
        msg_left.data = encoder_count_left;
        encoder_right_publisher_->publish(msg_right);
        encoder_left_publisher_->publish(msg_left);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr encoder_right_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr encoder_left_publisher_;
    int pi;
    int MPU9250_HANDLE;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}
