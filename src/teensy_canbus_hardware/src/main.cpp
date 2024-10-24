#include <FlexCAN_T4.h>
#include <math.h>
#include <Arduino.h>
#include <mks42d_can.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>


#define CCW 0
#define CW 1


#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;


//Define publisher of the motor
rcl_publisher_t pub_rpm;
rcl_publisher_t pub_pulse;
rcl_publisher_t pub_angle;


//Define subscriber of the motor
rcl_subscription_t motor_rpm;
rcl_subscription_t angle_motor;
rcl_subscription_t accel_motor;
rcl_subscription_t id_can;



std_msgs__msg__Int32 msg;

CAN_message_t msg_sent;
CAN_message_t msg_receive;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


struct motor_values {
  double angle;   // in radians
  double rpm;     // angular speed of the motor
  double accel;   // acceleration of the motor
  int id;
  double dir;
  uint8_t status;
  double rev;    //revolution of the motor for position control
  int pulse;
};

motor_values motor1;

void setup() {
  // Initialize CAN bus and serial communication
  can1.begin();
  can1.setBaudRate(250000);
  Serial.begin(115200);
  motor1.angle = getEncoderVal(0x01); // Get the initial encoder value

}

void loop() {

//Give command to move the motor  //Code didnt work
  // moverpm(0x01,CW,1600,200);
  // movetoposition(0x01,CCW,2,200,5,50);   

  //Reading from motor
  motor1.angle = getEncoderVal(0x01); // Update the angle
  Serial.print("Angle (radians): ");
  Serial.println(motor1.angle);

  motor1.rpm = getEncoderRpm(0x01);
  Serial.print("Motor speed (RPM): ");
  Serial.println(motor1.rpm);

  motor1.pulse = getEncoderPulse(0x01);
  Serial.print("No Pulse : ");
  Serial.println(motor1.pulse);

  calibrate(0x01);

}