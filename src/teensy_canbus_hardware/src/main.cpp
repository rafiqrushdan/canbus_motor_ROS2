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
void movetoposition(uint8_t can_id, uint8_t dir, uint8_t rev, uint16_t speed, uint8_t acc, uint32_t pulses);
void setDir();
void moverpm(uint8_t id, uint8_t dir, uint16_t speed, uint8_t accel);
double getEncoderVal(uint8_t can_id);
void calibrate(uint8_t can_id);
double returnEncoderRad(CAN_message_t recievedMsg); // Prototype for the new function

void setup() {
  // Initialize CAN bus and serial communication
  can1.begin();
  can1.setBaudRate(250000);
  Serial.begin(115200);
  motor1.angle = getEncoderVal(0x01); // Get the initial encoder value
}

void loop() {
  motor1.angle = getEncoderVal(0x01); // Update the angle
  Serial.print("Angle: ");
  Serial.println(motor1.angle);

  // Example functions you mentioned
  // calibrate(0x01);   // This works
  // movetoposition(0x01, 1, 2, 1023, 50, 5000); // Uncomment for testing
  // moverpm(0x01, CCW, 1023, 50);  // Uncomment for testing
  delay(100);
}

// Function that reads encoder value from CAN and converts it to radians
double getEncoderVal(uint8_t can_id) {
  double angle;

  // Send request to get the encoder value
  msg_sent.id = can_id;
  msg_sent.len = 2;
  msg_sent.buf[0] = 0x31;
  msg_sent.buf[1] = 0x32; // Fixed buffer access
  can1.write(msg_sent);

  // Read the response from the motor
  if (can1.read(msg_receive)) {
    // Check if response is valid before accessing buffer
    
    // Call returnEncoderRad to process the message and return the angle in radians
    return returnEncoderRad(msg_receive);
  }
  return -1; // Error case if no data is received
}

double returnEncoderRad(CAN_message_t recievedMsg) {
  // Extract encoder value from CAN message
  int32_t encoderVal = recievedMsg.buf[2];  // Starting from byte 2 (assumed to be the first byte of encoder data)
  encoderVal = (encoderVal << 8) | recievedMsg.buf[3];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[4];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[5];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[6];

  // Since it's a 32-bit value, we need to sign-extend it (assuming encoder uses 24 bits for signed data)
  if (encoderVal & 0x800000) { // Check if the sign bit (bit 23) is set
    encoderVal |= 0xFF000000;  // Sign extend the value to preserve the negative value
  }

  // Convert encoder value to radians (adjust the conversion factor if needed)
  return ((encoderVal / double(0x2000)) * PI/2); // Assuming 0x2000 corresponds to half a rotation
}


// Calibrate the motor
void calibrate(uint8_t can_id) {
  // Input to the motor
  msg_sent.id = can_id;
  msg_sent.len = 3;
  msg_sent.buf[0] = 0x80;
  msg_sent.buf[1] = 0;
  msg_sent.buf[2] = 0x81; // CRC
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

// Move motor to RPM
void moverpm(uint8_t id, uint8_t dir, uint16_t speed, uint8_t accel) {
  uint8_t status;

  // Input to the motor
  msg_sent.id = id;
  msg_sent.len = 8;
  msg_sent.buf[0] = 0xF6;
  msg_sent.buf[1] = (dir << 7) | (speed >> 8);
  msg_sent.buf[2] = speed & 0xFF;
  msg_sent.buf[3] = accel;
  msg_sent.buf[4] = 0x55; // Demo CRC or checksum
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

// Move to position function
void movetoposition(uint8_t can_id, uint8_t dir, uint8_t rev, uint16_t speed, uint8_t acc, uint32_t pulses) {
  uint8_t status;

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
        Serial.println("Run complete");
        break;
      case 3:
        Serial.println("End Limit Stopped");
        break; // Add missing break statement
    }
  }
}
