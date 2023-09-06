#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "camera_pins.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/time.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/compressed_image.h> // For image transport

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

geometry_msgs__msg__Twist msg;
rcl_subscription_t subscriber;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

sensor_msgs__msg__CompressedImage pub_msg; // Using CompressedImage for image transport
rcl_publisher_t publisher;

// ESP32-CAM configuration
const char *ssid = "Jhelum.net [Luqman House]";
const char *password = "7861234786";

void startCameraServer();

#define PIN_LEFT_FORWARD 14
#define PIN_LEFT_BACKWARD 27
#define PIN_RIGHT_FORWARD 26
#define PIN_RIGHT_BACKWARD 25

#define PWM_FREQUENCY 50

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
void motor1_action(int move_indicator, int pwm);
void motor2_action(int move_indicator, int pwm);
void stop();

// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

void twist_callback(const void *msgin)
{

  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  printf("Received: %f\n", msg->linear.x);
  // forward
  if (msg->linear.x > 0.4 && msg->angular.z < 0.1 && msg->angular.z > -0.1)
  {
    motor1_action(1, 10000);
    motor2_action(1, 10000);
  }
  // backward
  else if (msg->linear.x < -0.4 && msg->angular.z < 0.1 && msg->angular.z > -0.1)
  {
    motor1_action(0, 10000);
    motor2_action(0, 10000);
  }
  // left
  else if (msg->linear.x < 0.1 && msg->linear.x > -0.1 && msg->angular.z > 0.48)
  {
    motor1_action(0, 10000);
    motor2_action(1, 10000);
  }
  // right
  else if (msg->linear.x < 0.1 && msg->linear.x > -0.1 && msg->angular.z < -0.48)
  {
    motor1_action(1, 10000);
    motor2_action(0, 10000);
  }
  // forward left
  else if (msg->linear.x < 0.4 && msg->linear.x > 0.1 && msg->angular.z > 0.0)
  {
    motor1_action(1, 1000);
    motor2_action(1, 10000);
  }

  // forward right
  else if (msg->linear.x < 0.4 && msg->linear.x > 0.1 && msg->angular.z < 0.0)
  {
    motor1_action(1, 10000);
    motor2_action(1, 1000);
  }

  // backward left
  else if (msg->linear.x > -0.4 && msg->linear.x < -0.1 && msg->angular.z > 0.0)
  {
    motor1_action(0, 1000);
    motor2_action(0, 10000);
  }
  // backward right
  else if (msg->linear.x > -0.4 && msg->linear.x < -0.1 && msg->angular.z < 0.0)
  {
    motor1_action(0, 10000);
    motor2_action(0, 1000);
  }

  // stop
  else
  {
    stop();
  }
}

void motor1_action(int move_indicator, int pwm)
{
  analogWrite(PWM_FREQUENCY, pwm);
  if (move_indicator > 0)
  {
    digitalWrite(PIN_LEFT_FORWARD, HIGH);
    digitalWrite(PIN_LEFT_BACKWARD, LOW);
  }
  else
  {
    digitalWrite(PIN_LEFT_FORWARD, LOW);
    digitalWrite(PIN_LEFT_BACKWARD, HIGH);
  }
}

void motor2_action(int move_indicator, int pwm)
{
  analogWrite(PWM_FREQUENCY, pwm);
  if (move_indicator > 0)
  {
    digitalWrite(PIN_RIGHT_FORWARD, HIGH);
    digitalWrite(PIN_RIGHT_BACKWARD, LOW);
  }
  else
  {
    digitalWrite(PIN_RIGHT_FORWARD, LOW);
    digitalWrite(PIN_RIGHT_BACKWARD, HIGH);
  }
}

void stop()
{
  digitalWrite(PIN_RIGHT_FORWARD, 0);
  digitalWrite(PIN_RIGHT_BACKWARD, 0);
  digitalWrite(PIN_RIGHT_FORWARD, 0);
  digitalWrite(PIN_RIGHT_BACKWARD, 0);
  analogWrite(PWM_FREQUENCY, 0);
  analogWrite(PWM_FREQUENCY, 0);
}

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  Serial.setDebugOutput(true);
  Serial.println();

  // Set up camera configuration (same as your original setup)
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    if (psramFound())
    {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }
  else
  {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Initialize WiFi and connect to your network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set up ROS 2
  set_microros_serial_transports(Serial);

  Wire.begin();

  pinMode(PIN_LEFT_FORWARD, OUTPUT);
  pinMode(PIN_LEFT_BACKWARD, OUTPUT);
  pinMode(PIN_RIGHT_FORWARD, OUTPUT);
  pinMode(PIN_RIGHT_BACKWARD, OUTPUT);
  pinMode(PWM_FREQUENCY, OUTPUT);
  pinMode(PWM_FREQUENCY, OUTPUT);
  analogWrite(PWM_FREQUENCY, abs(0));
  analogWrite(PWM_FREQUENCY, abs(0));

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg,
      &twist_callback, ON_NEW_DATA));

  // Create a publisher for the video stream
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, pub_msg, CompressedImage),
      "video_stream"));

  startCameraServer();
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

// Assuming you have a valid rclcpp::Node defined
rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("your_node_name");

// Create a publisher for CompressedImage messages
auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>("your_image_topic", 10);

// Function to publish video frames as CompressedImage messages
void publishVideoFrame(
    const uint8_t *frame_data,
    size_t frame_size,
    const char *image_encoding) {

  auto pub_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
  pub_msg->header.stamp = node->now(); // Get the current time
  pub_msg->header.frame_id = "camera_frame"; // Replace with your frame ID
  pub_msg->height = 480; // Replace with your frame height
  pub_msg->width = 640; // Replace with your frame width
  pub_msg->format = image_encoding;

  pub_msg->data.assign(frame_data, frame_data + frame_size);

  publisher->publish(std::move(pub_msg));
}



void loop()
{
  delay(100);
  // Replace the following with your actual frame data, size, encoding, and timestamp
  // Handle ROS 2 events
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
