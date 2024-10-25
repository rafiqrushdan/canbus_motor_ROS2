#include <std_msgs/msg/int32.h>
#include <map>

std_msgs__msg__Int32 msg;

std::map<int, motor_values> motors;  // Map CAN_ID to motor values

bool calibrate_flag = false;
int8_t can_id = -1;  // Global variable to hold the latest can_id


//Starting of the callback function
void rpmMotor_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
}

void dirMotor_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
}

void accelMotor_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
}

void pulseMotor_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  
}

void idMotor_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int can_id = msg->data;

    if (motors.find(can_id) == motors.end()) {
        // Create a new motor if it doesn't exist
        motor_values new_motor;
        motors[can_id] = new_motor;
    }
    calibrate_flag = true;  // Set flag to trigger calibration
}

void revMotor_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
}