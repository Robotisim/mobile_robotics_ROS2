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
    // Convert the image to grayscale
    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(480, 480));

    cv::Mat gray_image;
    cv::cvtColor(resized_image, gray_image, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise
    cv::Mat blurred_image;
    cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 0);

    // Apply the Canny edge detection algorithm
    cv::Mat edges;
    cv::Canny(blurred_image, edges, 50, 150); // These thresholds might need to be adjusted

    // Display the Canny edges
    cv::imshow("edges", edges);
    cv::waitKey(1);

    // Apply the Hough Line Transform to find the longest straight lines
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edges, lines, 1, CV_PI/180, 150, 0, 0 ); // These parameters might need to be adjusted

    // Draw the longest straight lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line( image, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }

    // Display the image with lines


    // Display the image with lines
    // cv::imshow("lines", resized_image);
    // cv::waitKey(1);
}


    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received image message");

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

