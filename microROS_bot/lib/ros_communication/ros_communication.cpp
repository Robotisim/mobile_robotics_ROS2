#include "ros_communication.h"
#include "motor_control.h"

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


RosCommunication::RosCommunication(){
}

void RosCommunication::initialize(){
    Serial.begin(115200);
    // //Serial.println("ROS Communication node started");

    set_microros_serial_transports(Serial);

    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "cmd_vel_sub", "", &support);
}


void RosCommunication::executors_start(){
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,&RosCommunication::cmd_vel_callback, ON_NEW_DATA);

//   //Serial.println("Executors Started");
}
void RosCommunication::subscriber_publisher_define(){

    rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

     rclc_publisher_init_default(
        &left_encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "left_enc");

    // Initialize right encoder publisher
    rclc_publisher_init_default(
        &right_encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "right_enc");

}

void RosCommunication::publish_encoder_counts() {
    std_msgs__msg__Int16 left_msg;
    std_msgs__msg__Int16 right_msg;

    left_msg.data = get_enc_left();   // Assuming count_L is the left encoder count
    right_msg.data = get_enc_right();  // Assuming count_R is the right encoder count

    rcl_publish(&left_encoder_pub, &left_msg, NULL);
    rcl_publish(&right_encoder_pub, &right_msg, NULL);
}
void RosCommunication::cmd_vel_callback(const void *msg_recv){
    const geometry_msgs__msg__Twist * recieved_data = (const geometry_msgs__msg__Twist *) msg_recv ;
    float linear_vel = recieved_data->linear.x;
    float angular_vel = (recieved_data->angular.z)/2;


    // //Serial.print(linear_vel);//Serial.print(" / ");//Serial.println(angular_vel);

    if(linear_vel > 0) {
        //Serial.println("Forward");
        forward(linear_vel * 255);
    } else if(linear_vel < 0) {
        //Serial.println("Reverse");
        reverse(-linear_vel * 255);
    } else if(angular_vel < 0) {
        //Serial.println("right");
        right(-angular_vel *255);

    } else if(angular_vel > 0) {
        //Serial.println("left");
        left(angular_vel *255);
    } else {
        //Serial.println("Stop");
        stop();
    }




}
void RosCommunication::start_receiving_msgs(){
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        delay(100);
}