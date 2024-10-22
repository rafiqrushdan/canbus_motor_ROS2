#include <FlexCAN_T4.h>
#include <math.h>
#include <Arduino.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg;

struct motor_values {
  double radians = 0;
  bool enabled = false;
  int run_status = 0;
};

motor_values motor1;

// Function Prototypes
void getEncoderVal();
double returnEncoderRad(CAN_message_t receivedMsg);
void goHome();
void runToPosition(long encoder_counts, uint16_t speed, uint8_t acc);
void setMotorEnable(bool status);

void setup() {
  // Initialize CAN bus and serial communication
  can1.begin();
  can1.setBaudRate(250000);
  Serial.begin(115200);
  getEncoderVal(); // Get the initial encoder value
}

void loop() {
  // Check if a CAN message is received
  if (can1.read(msg)) {
    Serial.print("CAN1 "); 
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (uint8_t i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(msg.timestamp);

    // Process received CAN message
    switch (msg.buf[0]) {
      case 0x31:  // Encoder value
        motor1.radians = returnEncoderRad(msg);
        break;
      case 0xF3:  // Motor enable/disable status
        motor1.enabled = (msg.buf[1] == 0x01);
        break;
      case 0xF5:  // Motor run status
        motor1.run_status = msg.buf[1]; // 0 = fail, 1 = starting, 2 = complete, 3 = end limit stop
        break;
    }

    // delay(100); // Short delay to avoid message flooding
    // getEncoderVal(); // Continuously request the encoder value
      runToPosition(5000, 1500, 50); // Move to encoder position 5000, at speed 1500, with acceleration 50
      delay(5000); // Wait for 5 seconds before the next action
    
  }
}

void goHome() {
  CAN_message_t msgSend;
  msgSend.len = 2;
  msgSend.id = 0x01;
  msgSend.buf[0] = 0x91; // Go home command
  msgSend.buf[1] = 0x92; // CRC

  can1.write(msgSend);
}

void runToPosition(long encoder_counts, uint16_t speed, uint8_t acc) { 
  CAN_message_t msgSend;
  speed = constrain(speed, 0, 3000);
  msgSend.len = 8;
  msgSend.id = 0x01;
  msgSend.buf[0] = 0xF5; // Run motor to absolute position mode
  msgSend.buf[1] = speed >> 8;
  msgSend.buf[2] = speed & 0xFF;
  msgSend.buf[3] = acc; // Acceleration
  msgSend.buf[4] = encoder_counts >> 16;
  msgSend.buf[5] = encoder_counts >> 8;
  msgSend.buf[6] = encoder_counts & 0xFF;

  // CRC calculation
  uint8_t crc = msgSend.id;
  for (int i = 0; i < 7; i++) {
    crc += msgSend.buf[i];
  }
  msgSend.buf[7] = crc;

  can1.write(msgSend);
}

void getEncoderVal() {
  CAN_message_t msgSend;
  msgSend.len = 2;
  msgSend.id = 0x01;
  msgSend.buf[0] = 0x31; // Request encoder value
  msgSend.buf[1] = 0x32; // Checksum

  can1.write(msgSend);
}

void setMotorEnable(bool status) {
  CAN_message_t msgSend;
  msgSend.len = 3;
  msgSend.id = 0x01;
  msgSend.buf[0] = 0xF3; // Enable/Disable motor
  msgSend.buf[1] = status ? 0x01 : 0x00;
  msgSend.buf[2] = status ? 0xF5 : 0xF4; // CRC

  can1.write(msgSend);
}

double returnEncoderRad(CAN_message_t receivedMsg) {
  uint32_t encoderVal = receivedMsg.buf[2]; // Start assembling the 32-bit value
  encoderVal = (encoderVal << 8) | receivedMsg.buf[3];
  encoderVal = (encoderVal << 8) | receivedMsg.buf[4];
  encoderVal = (encoderVal << 8) | receivedMsg.buf[5];

  // Convert encoder value to radians (assuming 8192 ticks per revolution)
  return (double(encoderVal) / 8192.0) * 2 * PI;
}
