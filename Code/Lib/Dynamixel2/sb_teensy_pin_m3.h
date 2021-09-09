#ifndef SB_TEENSY_PIN_M3_H
#define SB_TEENSY_PIN_M3_H

extern const int L298N_IN1[3];
extern const int L298N_IN2[3];
extern const int ENC_A[3];
extern const int ENC_B[3];
extern const int PWM_EN_PIN[3];


extern float Setpoint[3], Input[3], Output[3];
extern volatile int lastEncoded[3];
extern volatile long encoderValue[3];
extern long lastencoderValue[3];
extern int lastMSB[3];
extern int lastLSB[3];

void updateEncoder0();
void updateEncoder1();
void updateEncoder2();

#endif