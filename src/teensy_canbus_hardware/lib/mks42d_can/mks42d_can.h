#ifndef MKS42D_CAN_H
#define MKS42D_CAN_H

#include <stdint.h>  // For standard integer types
#include <FlexCAN_T4.h>


// Function declarations

// Function to get encoder value in radians
double getEncoderVal(uint8_t can_id);

// Function to convert the received CAN message into radians
double returnEncoderRad(CAN_message_t recievedMsg);

// Function to get encoder RPM value
double getEncoderRpm(uint8_t can_id);

// Function to convert the received CAN message into RPM
double returnEncoderRpm(CAN_message_t recievedMsg);

// Function to calibrate the motor
void calibrate(uint8_t can_id);

// Function to move motor to a certain RPM
void moverpm(uint8_t id, uint8_t dir, uint16_t speed, uint8_t accel);

// Function to move motor to a certain position
void movetoposition(uint8_t can_id, uint8_t dir, uint8_t rev, uint16_t speed, uint8_t acc, uint32_t pulses);

// Function to get encoder pulse count
int getEncoderPulse(uint8_t can_id);

// Function to convert the received CAN message into encoder pulse
int returnEncoderPulse(CAN_message_t recievedMsg);

#endif /* MKS42D_CAN_H */
