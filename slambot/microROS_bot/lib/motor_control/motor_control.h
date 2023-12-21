#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Pin Definitions
#define motor_right_b 19
#define motor_right_a 21
#define motor_left_a 18
#define motor_left_b 5
#define enc_RA  13
#define enc_RB  14
#define enc_LA  27
#define enc_LB  12
// Channels
#define motor_right_pwm 32
#define motor_left_pwm 33
#define pwm_channel_mr 0
#define pwm_channel_ml 1

// Functions
void forward(int speed);
void stop();
void left(int speed);
void right(int speed);
void reverse(int speed);
void motor_setup();
void Update_encR();
void Update_encL();
int get_enc_left();
int get_enc_right();
#endif
