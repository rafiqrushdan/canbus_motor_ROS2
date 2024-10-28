#include "mks42d_can.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg_sent;
CAN_message_t msg_receive;

// Function definitions

double getEncoderVal(uint8_t can_id) {
  // Send request to get the encoder value
  msg_sent.id = can_id;
  msg_sent.len = 2;
  msg_sent.buf[0] = 0x31;
  msg_sent.buf[1] = 0x32; // Command to request encoder position
  can1.write(msg_sent);

  // Read the response from the motor
  if (can1.read(msg_receive)) {
    return returnEncoderRad(msg_receive); // Process the message and return the angle in radians
  }
  return -1; // Error case if no data is received
}

double returnEncoderRad(CAN_message_t recievedMsg) {
  // Extract encoder value from CAN message
  int32_t encoderVal = recievedMsg.buf[2];  // Starting from byte 2
  encoderVal = (encoderVal << 8) | recievedMsg.buf[3];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[4];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[5];
  encoderVal = (encoderVal << 8) | recievedMsg.buf[6];

  // Sign extension for 24-bit encoder value
  if (encoderVal & 0x800000) { // Check if the sign bit (bit 23) is set
    encoderVal |= 0xFF000000;  // Sign extend the value
  }

  // Convert encoder value to radians (adjust the conversion factor if needed)
  return ((encoderVal / double(0x2000)) * PI / 2); // Assuming 0x2000 corresponds to half a rotation
}

double getEncoderRpm(uint8_t can_id) {
  // Send request to get the RPM value
  msg_sent.id = can_id;
  msg_sent.len = 2;
  msg_sent.buf[0] = 0x32;
  msg_sent.buf[1] = 0x33; // Command to request RPM
  can1.write(msg_sent);

  // Read the response from the motor
  if (can1.read(msg_receive)) {
    return returnEncoderRpm(msg_receive); // Process the message and return RPM
  }
  return -1; // Error case if no data is received
}

double returnEncoderRpm(CAN_message_t recievedMsg) {
  // Extract encoder RPM from CAN message
  int16_t encoderspeed = recievedMsg.buf[2];  // Starting from byte 2
  encoderspeed = (encoderspeed << 8) | recievedMsg.buf[3];

  return encoderspeed; // Return the RPM value (positive or negative)
}

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

void moverpm(uint8_t id, uint8_t dir,u_int16_t rev, uint16_t speed, uint8_t accel) {
  uint8_t status;

  // Input to the motor
  msg_sent.id = id;
  msg_sent.len = 8;
  msg_sent.buf[0] = 0xF6;
  msg_sent.buf[1] = (dir << 8) | (rev << 7)|(speed << 5);
  msg_sent.buf[2] = speed ;
  msg_sent.buf[3] = accel;
  msg_sent.buf[4] = 0x55; // Demo CRC or checksum
  can1.write(msg_sent);

  // Output from the motor (only get the status)
  if (can1.read(msg_receive)) {
    status = msg_receive.buf[1];

    switch (status) {
      case 1:
        Serial.println("Stop motor success");
        break;
      case 0:
        Serial.println("Stop motor failed");
        break;
      case 2:
        Serial.println("Stop motor success");
        break;
    }
  }
}

void moverpm(uint8_t id, uint8_t dir,u_int16_t rev, uint16_t speed, uint8_t accel) {
  uint8_t status;

  // Input to the motor
  msg_sent.id = id;
  msg_sent.len = 8;
  msg_sent.buf[0] = 0xF6;
  msg_sent.buf[1] = (dir << 8) | (rev << 7)|(speed << 5);
  msg_sent.buf[2] = speed ;
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

//this code used position mode 1 
void movetoposition(uint8_t can_id, uint8_t dir, uint8_t rev, uint16_t speed, uint8_t acc, uint32_t pulses) {
  uint8_t status;

  // Prepare CAN message with 8 bytes
  msg_sent.id = can_id;       // Set CAN ID (e.g., 0x01)
  msg_sent.len = 8;           // DLC is 8 bytes
  msg_sent.buf[0] = 0xFD;     // Command code (FD)
  
  // byte 2: direction and revolutions (dir in b7, rev in b6-b4)
  msg_sent.buf[1] = (dir << 7) | (rev << 6) | (speed << 3);
  
  // byte 3: speed (bits b7-b0)
  speed = (speed<< 8)|msg_sent.buf[2];
  msg_sent.buf[3] = acc;         

  // bytes 5-7: pulses (23 bits -> 3 bytes)
  pulses = (pulses << 8) | msg_sent.buf[4] ; 
  pulses = (pulses << 8) | msg_sent.buf[4] ;  
  pulses = (pulses <<8)  | msg_sent.buf[4] ;      
  
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
        break;
    }
  }
}

int getEncoderPulse(uint8_t can_id) {
  // Send request to get the encoder pulse
  msg_sent.id = can_id;
  msg_sent.len = 2;
  msg_sent.buf[0] = 0x33;
  msg_sent.buf[1] = 0x34; // Command to request encoder position
  can1.write(msg_sent);

  if (can1.read(msg_receive)) {
    return returnEncoderPulse(msg_receive); // Process the message and return the angle in radians
  }
  return -1; // Error case if no data is received
}

int returnEncoderPulse(CAN_message_t recievedMsg) {
  // Extract encoder pulse from CAN message
  int32_t encoderpulse = recievedMsg.buf[1];  // Starting from byte 2
  encoderpulse = (encoderpulse << 8) | recievedMsg.buf[2];
  encoderpulse = (encoderpulse << 8) | recievedMsg.buf[3];
  encoderpulse = (encoderpulse << 8) | recievedMsg.buf[4];

  return encoderpulse; // Return the encoder pulse value (positive or negative)
}
