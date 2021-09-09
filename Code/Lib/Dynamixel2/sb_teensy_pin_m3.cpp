
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "sb_teensy_pin_m3.h"



const int L298N_IN1[3] = {15, 36, 34};
const int L298N_IN2[3] = {14, 37, 33};
const int PWM_EN_PIN[3] = {16, 38, 35};
const int ENC_A[3] = {23, 20, 19};
const int ENC_B[3] = {22, 21, 18}; 





float Setpoint[3], Input[3], Output[3];
volatile int lastEncoded[3] = {0, 0, 0};
volatile long encoderValue[3] = {0, 0, 0};
long lastencoderValue[3] = {0, 0, 0};
int lastMSB[3] = {0, 0, 0};
int lastLSB[3] = {0, 0, 0};


void updateEncoder0(){
  int MSB = digitalRead(ENC_A[0]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[0]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[0] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[0] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[0] --;

  lastEncoded[0] = encoded; //store this value for next time
  Input[0] = encoderValue[0];
  // Serial1.println(encoderValue[0]);
}

void updateEncoder1(){
  int MSB = digitalRead(ENC_A[1]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[1]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[1] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[1] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[1] --;

  lastEncoded[1] = encoded; //store this value for next time
  Input[1] = encoderValue[1];
  // Serial.println(encoderValue[1]);
}

void updateEncoder2(){
  int MSB = digitalRead(ENC_A[2]); //MSB = most significant bit
  int LSB = digitalRead(ENC_B[2]); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[2] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[2] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[2] --;

  lastEncoded[2] = encoded; //store this value for next time
  Input[2] = encoderValue[2];
}

