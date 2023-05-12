#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor()
        : Node("image_processor")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robot_a/camera/image_raw", default_qos, std::bind(&ImageProcessor::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
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

        // Convert the image to grayscale
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        // Display the image
        cv::imshow("view", gray_image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
