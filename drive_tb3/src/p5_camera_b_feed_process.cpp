#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor()
        : Node("image_processor")
    {
        RCLCPP_INFO(this->get_logger(),"Starting image processor node");

        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robot_a/camera/image_raw", default_qos, std::bind(&ImageProcessor::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot_a/cmd_vel", 10);
    }

private:
    void process_image(cv::Mat &image)
    {
        // Threshold the image to isolate the walls
        cv::Mat binary_image;
        cv::inRange(image, cv::Scalar(0, 0, 0), cv::Scalar(80, 80, 80), binary_image);

        // Apply Gaussian blur to reduce noise
        cv::GaussianBlur(binary_image, binary_image, cv::Size(5, 5), 0);

        // Find the contours of the walls
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        // Process the contours to find the largest one
        double max_area = 0;
        std::vector<cv::Point> largest_contour;
        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > max_area)
            {
                max_area = area;
                largest_contour = contour;
            }
        }

        // Draw the largest contour
        cv::Mat drawing = cv::Mat::zeros(binary_image.size(), CV_8UC3);
        cv::drawContours(drawing, std::vector<std::vector<cv::Point>>{largest_contour}, 0, cv::Scalar(255, 255, 255), 2);

        // Find the centroid of the largest contour
        cv::Moments m = cv::moments(largest_contour);
        int cx = int(m.m10 / m.m00);
        int cy = int(m.m01 / m.m00);

        // Draw the centroid
        cv::circle(drawing, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);

        // Display the processed image
        cv::imshow("processed", drawing);

        // Calculate the control command based on the centroid's position
        float error = float(image.cols / 2 - cx);
        float Kp = 0.001;
        float angular_velocity = Kp * error;
        float linear_velocity = 0.1;

        // Send the control command to the robot
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
        // pub_->publish(cmd_vel);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image message");

        // Convert the ROS image into an OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
                catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Process the image to navigate the maze
        process_image(cv_ptr->image);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}

