#include <Arduino.h>
#include "ros_communication.h"
#include "motor_control.h"
RosCommunication ros_communication ;

unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 300;  // Interval for publishing encoder counts

void setup() {
    motor_setup();
    ros_communication.initialize();
    ros_communication.subscriber_publisher_define();
    ros_communication.executors_start();
}

void loop() {
     unsigned long currentTime = millis();
    if (currentTime - lastPublishTime >= publishInterval) {
        lastPublishTime = currentTime;
        ros_communication.publish_encoder_counts();
    }
    ros_communication.start_receiving_msgs();

}