#include "motor_control.h"
int count_R = 0; // For Encoders
int count_L = 0;


void motor_setup(){
    // Motor pinout definition
  pinMode(motor_left_a, OUTPUT);
  pinMode(motor_left_b, OUTPUT);
  pinMode(motor_right_a, OUTPUT);
  pinMode(motor_right_b, OUTPUT);

  // Encoder pinout definition
  pinMode(enc_RA, INPUT);
  pinMode(enc_RB, INPUT);
  pinMode(enc_LA, INPUT);
  pinMode(enc_LB, INPUT);

  ledcSetup(pwm_channel_mr , 5000, 8 ); //2^8 = 256 , 0-255
  ledcSetup(pwm_channel_ml , 5000, 8 ); //2^8 = 256 , 0-255


  ledcAttachPin(motor_right_pwm,pwm_channel_mr);
  ledcAttachPin(motor_left_pwm,pwm_channel_ml);

  // Interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(enc_RA), Update_encR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_LA), Update_encL, CHANGE);


}
void Update_encR() {
  if (digitalRead(enc_RA) == digitalRead(enc_RB)) count_R--;
  else count_R++;
}

void Update_encL() {
  if (digitalRead(enc_LA) == digitalRead(enc_LB)) count_L--;
  else count_L++;
}

int get_enc_left(){
  return count_L;
}
int get_enc_right(){
  return count_R;
}

void forward(int speed) {
  digitalWrite(motor_right_a, HIGH);
  digitalWrite(motor_right_b, LOW);
  digitalWrite(motor_left_a, HIGH);
  digitalWrite(motor_left_b, LOW);
  ledcWrite(pwm_channel_mr, speed);
  ledcWrite(pwm_channel_ml, speed);
}

void reverse(int speed) {
  digitalWrite(motor_right_a, LOW);
  digitalWrite(motor_right_b, HIGH);
  digitalWrite(motor_left_a, LOW);
  digitalWrite(motor_left_b, HIGH);
  ledcWrite(pwm_channel_mr, speed);
  ledcWrite(pwm_channel_ml, speed);
}

void right(int speed) {
  digitalWrite(motor_right_a, LOW);
  digitalWrite(motor_right_b, HIGH);
  digitalWrite(motor_left_a, HIGH);
  digitalWrite(motor_left_b, LOW);
  ledcWrite(pwm_channel_mr, speed);
  ledcWrite(pwm_channel_ml, speed);
}

void left(int speed) {
  digitalWrite(motor_right_a, HIGH);
  digitalWrite(motor_right_b, LOW);
  digitalWrite(motor_left_a, LOW);
  digitalWrite(motor_left_b, HIGH);
  ledcWrite(pwm_channel_mr, speed);
  ledcWrite(pwm_channel_ml, speed);
}

void stop() {
  digitalWrite(motor_right_a, LOW);
  digitalWrite(motor_right_b, LOW);
  digitalWrite(motor_left_a, LOW);
  digitalWrite(motor_left_b, LOW);
  ledcWrite(pwm_channel_mr, 0);
  ledcWrite(pwm_channel_ml, 0);
}


