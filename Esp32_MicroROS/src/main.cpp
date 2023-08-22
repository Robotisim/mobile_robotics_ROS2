#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include "AS5600.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/time.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

geometry_msgs__msg__Twist msg;
rcl_subscription_t subscriber;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_publisher_t publisher;
std_msgs__msg__Int32 pub_msg;
rcl_publisher_t encoder_publisher_0;
rcl_publisher_t encoder_publisher_1;
rcl_publisher_t imu_publisher;

AS5600 as5600_0(&Wire);
AS5600 as5600_1(&Wire1);

sensor_msgs__msg__Imu imu_msg;
Adafruit_MPU6050 mpu;

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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    imu_msg.angular_velocity.x = a.gyro.x;
    imu_msg.angular_velocity.y = a.gyro.y;
    imu_msg.angular_velocity.z = a.gyro.z;

  }
}

void twist_callback(const void *msgin)
{

  pub_msg.data = 1;
  rcl_publish(&publisher, &pub_msg, NULL);
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

void setupEncoder0()
{
  Wire.begin();
  if (!as5600_0.begin())
  {
    Serial.println("Encoder 0 not connected!");
    Wire.end();
  }
  else
  {
    as5600_0.setDirection(AS5600_CLOCK_WISE);
    Serial.println("Connect device 0: true");
  }
}

void setupEncoder1()
{
  Wire1.begin(4, 5);
  if (!as5600_1.begin())
  {
    Serial.println("Encoder 1 not connected!");
    Wire1.end();
  }
  else
  {
    as5600_1.setDirection(AS5600_CLOCK_WISE);
    Serial.println("Connect device 1: true");
  }
}

int resolution_0()
{
  if (!as5600_0.begin())
  {
  }
  else
  {
    Serial.print("Resolution 1: ");
    Serial.println(as5600_0.readAngle());
    int resolution0 = as5600_0.readAngle();
    return resolution0;
  }
}
int resolution_1()
{
  if (!as5600_1.begin())
  {
  }
  else
  {
    Serial.print("Resolution 2: ");
    Serial.println(as5600_1.readAngle());
    int resolution1 = as5600_1.readAngle();
    return resolution1;
  }
}

void setupMPU()
{
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void publishEncoderValues(rcl_publisher_t *encoder_publisher, int encoder_value)
{
  pub_msg.data = encoder_value;
  rcl_publish(encoder_publisher, &pub_msg, NULL);
}

void setup()
{
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  Wire.begin();
  setupEncoder0();
  setupEncoder1();
  setupMPU();

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

  // create node
  RCCHECK(rclc_node_init_default(&node, "imu_publisher_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // msg.data = 0;
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg,
      &twist_callback, ON_NEW_DATA));

  // Create publishers for each encoder
  RCCHECK(rclc_publisher_init_default(
      &encoder_publisher_0,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "encoder0_topic"));

  RCCHECK(rclc_publisher_init_default(
      &encoder_publisher_1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "encoder1_topic"));
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_info_topic"));

}

void loop()
{
  delay(100);
  int encoder_value_0 = resolution_0();                        // Read encoder value using your function
  publishEncoderValues(&encoder_publisher_0, encoder_value_0); // Publish encoder 0 value

  int encoder_value_1 = resolution_1();                        // Read the second encoder value using your function
  publishEncoderValues(&encoder_publisher_1, encoder_value_1); // Publish encoder 1 value


  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}