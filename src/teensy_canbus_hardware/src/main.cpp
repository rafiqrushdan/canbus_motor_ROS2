#include <FlexCAN_T4.h>
#include <math.h>
#include <Arduino.h>

#define CCW 0
#define CW 1

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg_sent;
CAN_message_t msg_receive;

struct motor_values {
  double angle;   // in radians
  double rpm;     // angular speed of the motor
  double accel;   // acceleration of the motor
  int id;
  double dir;
  uint8_t status;
  double rev;    //revolution of the motor for position control
};

motor_values motor1;

// Define the function prototype
void movetoposition();
void setDir();
void moverpm(uint8_t id, uint8_t dir, uint16_t speed, uint8_t accel);
double getEncoderVal(uint8_t can_id);
void calibrate(uint8_t can_id);

void setup() {
  // Initialize CAN bus and serial communication
  can1.begin();
  can1.setBaudRate(250000);
  Serial.begin(115200);
  motor1.angle = getEncoderVal(0x01); // Get the initial encoder value
}

void loop() {
  motor1.angle = getEncoderVal(0x01);
  Serial.print("Angle: ");
  Serial.println(motor1.angle);

  // calibrate(0x01);
  //movetoposition(0x01, 1, 2, 1023, 50, 5000)
  delay(100);
}

double getEncoderVal(uint8_t can_id) {
  double angle;

  // Send request to get the encoder value
  msg_sent.id = can_id;
  msg_sent.len = 2;
  msg_sent.buf[0] = 31;
  msg_sent.buf[7] = 32;
  can1.write(msg_sent);

  // Read the response from the motor
  if (can1.read(msg_receive)) {
    uint64_t value = 0;
    value |= (uint64_t)msg_receive.buf[1] << 40; // Byte 2
    value |= (uint64_t)msg_receive.buf[2] << 32; // Byte 3
    value |= (uint64_t)msg_receive.buf[3] << 24; // Byte 4
    value |= (uint64_t)msg_receive.buf[4] << 16; // Byte 5
    value |= (uint64_t)msg_receive.buf[5] << 8;  // Byte 6
    value |= (uint64_t)msg_receive.buf[6];       // Byte 7

    angle = (double)value / 1024 * PI; // Modify according to the encoder specification
    return angle;
  }
  return -1; // Error case if no data is received
}

void calibrate(uint8_t can_id) {
  // Input to the motor
  msg_sent.id = can_id;
  msg_sent.len = 3;
  msg_sent.buf[0] = 80;
  msg_sent.buf[1] = 0;
  msg_sent.buf[2] = 16; // CRC
  can1.write(msg_sent);

  // Output from the motor
  if (can1.read(msg_receive)) {
    uint8_t status = msg_receive.buf[1];

    switch (status) {
      case 0:
        Serial.println("Calibrating….");
        break;
      case 1:
        Serial.println("Calibrated successfully.");
        break;
      case 2:
        Serial.println("Calibration failed");
        break;
    }
  }
}

void moverpm(uint8_t id, uint8_t dir, uint16_t speed, uint8_t accel) {
  bool status;

  // Input to the motor
  msg_sent.id = id;
  msg_sent.len = 8;
  msg_sent.buf[0] = 0xF6;
  msg_sent.buf[1] = (dir << 7) | (speed >> 8);
  msg_sent.buf[2] = speed & 0xFF;
  msg_sent.buf[3] = accel;
  msg_sent.buf[4] = 155;
  can1.write(msg_sent);

  // Output from the motor (only get the status)
  if (can1.read(msg_receive)) {
    status = msg_receive.buf[1];

    switch (status) {
      case 1:
        Serial.println("Run success");
        break;
      case 0:
        Serial.println("Run failed");
        break;
    }
  }
}

void movetoposition(uint8_t can_id, uint8_t dir, uint8_t rev, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    bool status;

  // Prepare CAN message with 8 bytes
  msg_sent.id = can_id;       // Set CAN ID (e.g., 0x01)
  msg_sent.len = 8;           // DLC is 8 bytes
  msg_sent.buf[0] = 0xFD;     // Command code (FD)
  
  // byte 2: direction and revolutions (dir in b7, rev in b6-b4)
  msg_sent.buf[1] = (dir << 7) | ((rev & 0x07) << 4);
  
  // byte 3: speed (bits b7-b0)
  msg_sent.buf[2] = (speed >> 8) & 0xFF;  // Speed high byte
  msg_sent.buf[3] = speed & 0xFF;         // Speed low byte
  
  // byte 4: acceleration (bits b7-b0)
  msg_sent.buf[4] = acc;
  
  // bytes 5-7: pulses (23 bits -> 3 bytes)
  msg_sent.buf[5] = (pulses >> 16) & 0xFF;  // Pulses high byte
  msg_sent.buf[6] = (pulses >> 8) & 0xFF;   // Pulses mid byte
  msg_sent.buf[7] = pulses & 0xFF;          // Pulses low byte
  
  // byte 8: CRC (checksum)
  msg_sent.buf[8] = 255;

  // Send CAN message
  can1.write(msg_sent);

  // Output from the motor (only get the status)
  if (can1.read(msg_receive)) {
    status = msg_receive.buf[1];

    switch (status) {
      case 0:
        Serial.println("Run failed");
        break; 
      case 1:
        Serial.println("Run starting");
        break;
      case 2:
        Serial.println("Run complete ");
      case 3:
        Serial.println("End Limit Stopped")
    

    }
  }

}
