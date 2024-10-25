#include <math.h>
#include <Arduino.h>
#include <map>

//Library for can
#include <FlexCAN_T4.h>

//Library for executing ROS
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

//Custom library
#include "config_subscallback.h"
#include <mks42d_can.h>


#define CCW 0
#define CW 1


// #ifndef RCCHECK
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
// #endif
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// #define EXECUTE_EVERY_N_MS(MS, X)  do { \
//   static volatile int64_t init = -1; \
//   if (init == -1) { init = uxr_millis();} \
//   if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
// } while (0)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;


//Define publisher of the motor
rcl_publisher_t pub_rpm;
rcl_publisher_t pub_pulse;
rcl_publisher_t pub_angle;


//Define subscriber of the motor
rcl_subscription_t motor_rpm;
rcl_subscription_t accel_motor;
rcl_subscription_t id_can;
rcl_subscription_t dir_motor;
rcl_subscription_t pulse_motor;
rcl_subscription_t rev_motor;


std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 angle_msg;  // for publishing angle as float

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
  //Real time data (publish into ros)
  double angle;   // in radians
  double rpm;     // angular speed of the motor
  double accel;   // acceleration of the motor
  int pulse;

  //Subscribe to ROS
  int id;
  double dir;
  uint8_t status;
  double rev;    //revolution of the motor for position control
  int subs_pulse;
  int subs_rpm;
  int subs_acel;

};

motor_values motor1;

void setup() {
  // Initialize CAN bus and serial communication
  can1.begin();
  can1.setBaudRate(250000);
  Serial.begin(115200);
  motor1.angle = getEncoderVal(0x01); // Get the initial encoder value

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
  &motor_rpm,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  "motor_rpm"));

  RCCHECK(rclc_subscription_init_default(
  &dir_motor,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg, Int32),
  "dir_motor"));

  RCCHECK(rclc_subscription_init_default(
  &accel_motor,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32),
  "revolution_motor"));

  RCCHECK(rclc_subscription_init_default(
  &id_can,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32),
  "id_can"));

  RCCHECK(rclc_subscription_init_default(
  &rev_motor,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32),
  "rev_motor"));

  RCCHECK(rclc_subscription_init_default(
  &rev_motor,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32),
  "pulse_motor"));


  //create publisher 
  RCCHECK(rclc_publisher_init_default(
  &pub_rpm,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32),
  "publish_rpm"));

  RCCHECK(rclc_publisher_init_default(
  &pub_pulse,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32),
  "publish_pulse"));

  RCCHECK(rclc_publisher_init_default(
  &pub_angle,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),
  "publish_angle"));



// create executor 
RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
RCCHECK(rclc_executor_add_subscription(&executor, &motor_rpm, &msg, &rpmMotor_subscription_callback, ON_NEW_DATA));
RCCHECK(rclc_executor_add_subscription(&executor, &accel_motor, &msg, &accelMotor_subscription_callback, ON_NEW_DATA));
RCCHECK(rclc_executor_add_subscription(&executor, &dir_motor, &msg, &dirMotor_subscription_callback, ON_NEW_DATA));
RCCHECK(rclc_executor_add_subscription(&executor, &rev_motor, &msg, &revMotor_subscription_callback, ON_NEW_DATA));
RCCHECK(rclc_executor_add_subscription(&executor, &pulse_motor, &msg, &pulseMotor_subscription_callback, ON_NEW_DATA));
RCCHECK(rclc_executor_add_subscription(&executor, &id_can, &msg, &idMotor_subscription_callback, ON_NEW_DATA));



}

void loop() {

    for (auto& [can_id, motor] : motors) {
        // Read motor data
        motor.angle = getEncoderVal(can_id);  // Example function call for motor
        motor.rpm = getEncoderRpm(can_id);
        
        // Publish to ROS2 based on the motor's CAN ID
        std_msgs__msg__Int32 msg;
        msg.data = static_cast<int32_t>(motor.rpm);
        rcl_publish(&pub_rpm, &msg, NULL);

        msg.data =static_cast<float>(motor.angle);
        rcl_publish(&pub_angle,&msg,NULL);

        msg.data = static_cast<int32_t>(motor.pulse);
        rcl_publish(&pub_pulse, &msg, NULL);
        

        if (calibrate_flag) {
            calibrate(can_id);  // Calibrate specific motor
            calibrate_flag = false;
        }

        //moverpm(can_id,motor.dir,motor.rpm,motor.accel);
        //movetoposition(can_id,motor.dir,motor.rev,motor.speed,motor.pub_accel,motor.pub_pulse);
    }
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}


void error_loop() {
  while (1) {
    delay(100);
  }
}



