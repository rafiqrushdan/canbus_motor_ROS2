#include <FlexCAN_T4.h>
#include <math.h>
#include <Arduino.h>

#define CCW 0
#define CW 1

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

CAN_message_t msg_sent;
CAN_message_t msg_receive;

struct motor_values {
  double angle;   //in radians
  double rpm;     //get the angular speed of the motor
  double accel;   //get the acccelaration of the motor
  int id;
  double dir;
  uint8_t status;
};

motor_values motor1;

//Define the function prototype
void movetoposition();
void setDir();
void moverpm();
void getEncoderVal();
void calibrate();


void setup() {
  // Initialize CAN bus and serial communication
  can1.begin();
  can1.setBaudRate(250000);
  Serial.begin(115200);
  getEncoderVal(0x01); // Get the initial encoder value
}

void loop() {
getEncoderVal(0x01);
delay(100);

  
}

void getEncoderVal(u_int can_id)
{ //use addition instead of carry mode

  msg_sent.id=can_id;
  msg_sent.len=2;
  msg_sent.buf[0]=31;
  msg_sent.buf[1]=32;

  msg_receive.id=can_id;
  msg_receive.len=8;
  msg_receive.buf[0]=31;
  msg_receive.buf[7]=255;

  double value;

      value |= ((uint64_t)msg.buf[1]) << 40; // Byte 2
      value |= ((uint64_t)msg.buf[2]) << 32; // Byte 3
      value |= ((uint64_t)msg.buf[3]) << 24; // Byte 4
      value |= ((uint64_t)msg.buf[4]) << 16; // Byte 5
      value |= ((uint64_t)msg.buf[5]) << 8;  // Byte 6
      value |= ((uint64_t)msg.buf[6]);       // Byte 7

    

  motor1.angle=value/1024*PI;

  Serial.print("Angle:"); Serial.println(motor1.angle);
}

void calibrate()
{
  //Input to the motor
  msg_sent.id=motor1.id;
  msg_sent.len=3;
  msg_sent.buf[0]=80;
  msg_sent.buf[1]=0;
  msg_sent.buf[2]=16;      //CRC


  //Output from the motor
  msg_receive.id=motor1.id;
  msg_receive.len=3;
  msg_receive.buf[0]=80;
  uint8_t status= msg_receive.buf[1];
  msg_receive.buf[2]=16;
  
  switch(status)
  {
    case 0:
      Serial.println(" Calibrating….");
        break;
    case 1:
      Serial.println ("Calibrated success.");
        break;
    case 2:
       Serial.println("Calibrating fail");
       break;
  }

  can1.write(msg_sent);
  can1.read(msg_receive);
}

void moverpm(uint8_t id,uint dir,u_int16_t speed,uint accel )
{
  bool status;

  //Input to the motor
  msg_sent.id=id;
  msg_sent.len=5;
  msg_sent.buf[0]=0xF6;
  msg_sent.buf[1]=(dir << 7) | (speed>> 8);
  msg_sent.buf[2]=speed & 0xFF;
  msg_sent.buf[3]=accel;
  msg_sent.buf[4]=155;

  //Output of the motor (only get the status)
  msg_receive.id=motor1.id;
  msg_receive.len=3;
  msg_receive.buf[0]=0xF6;
  msg_receive.buf[1]=status;
  msg_receive.buf[2]=255;

  switch (status)
  {
  case 1:
    Serial.println("Run success");
    break;

  case 0:
    Serial.println("Run Failed");
    break;
  }
}