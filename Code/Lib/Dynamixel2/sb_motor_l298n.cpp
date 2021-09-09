
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "sb_motor_l298n.h"


SB_Motor_L298N::SB_Motor_L298N(int motorID, float ticksPerRev, float Kp, float Ki, float Kd,
	float* Input, float* Output, float* Setpoint): 
	myPID(Input + motorID, Output + motorID, Setpoint + motorID, Kp, Ki, Kd, DIRECT){
	
	m_motorID = motorID;
	m_ticksPerRev = ticksPerRev;
	m_Input = Input + motorID;
	m_Output = Output + motorID;
	m_Setpoint = Setpoint + motorID;
	m_Kp = Kp;
	m_Ki = Ki;
	m_Kd = Kd;

	m_L298N_IN1 = L298N_IN1[m_motorID];
	m_L298N_IN2 = L298N_IN2[m_motorID];
	m_encA = ENC_A[m_motorID];
	m_encB = ENC_B[m_motorID];
	m_PWM_EN_PIN = PWM_EN_PIN[m_motorID];

}

void SB_Motor_L298N::Init(float ticksPerRev, float Kp, float Ki, float Kd) {
	m_ticksPerRev = ticksPerRev;
	m_Kp = Kp;
	m_Ki = Ki;
	m_Kd = Kd;
	
	pinMode(m_PWM_EN_PIN, OUTPUT);
	pinMode(m_L298N_IN1, OUTPUT);
	pinMode(m_L298N_IN2, OUTPUT);
	pinMode(m_encA, INPUT_PULLUP);
	pinMode(m_encB, INPUT_PULLUP);

	myPID.SetTunings(m_Kp, m_Ki, m_Kd);


//	Serial.println("Pin mode set");


	switch (m_motorID) {
		case 0:
			attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder0, CHANGE);
			attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder0, CHANGE);
		break;
		case 1:
			attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder1, CHANGE);
			attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder1, CHANGE);
		break;
		case 2:
			attachInterrupt(digitalPinToInterrupt(m_encA), updateEncoder2, CHANGE);
			attachInterrupt(digitalPinToInterrupt(m_encB), updateEncoder2, CHANGE);
		break;
		default:
			Serial.println("init default");
			break;
	}
	// attachInterrupt(digitalPinToInterrupt(this->encA), updateEncoder, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(this->encB), updateEncoder, CHANGE);
//	Serial.println("Interrupt set");

	myPID.SetOutputLimits(-255, 255);
  	myPID.SetMode(AUTOMATIC);
  	myPID.SetSampleTime(1);
	//Serial.println("PID set");
  	m_init = true;

}

void SB_Motor_L298N::Update() {
	myPID.Compute();
	// Serial.println(*m_Output);
	pwmValN = static_cast<int>((abs(*m_Output) - *m_Output) / 2);
  	pwmValP = static_cast<int>((abs(*m_Output) + *m_Output) / 2);
  	
	digitalWrite(m_L298N_IN1, pwmValP);
	digitalWrite(m_L298N_IN2, pwmValN);
	analogWrite(m_PWM_EN_PIN, abs(*m_Output));
	m_lastSetpoint = *m_Setpoint;
}

void SB_Motor_L298N::Stop(bool ifStop) {
	m_stop = ifStop;
	if (m_stop) {
		analogWrite(m_L298N_IN1, 0);
		analogWrite(m_L298N_IN2, 0);
	}
}

/*void SB_Motor::Enable(bool ifEnable) {
	if (ifEnable) digitalWrite(m_PWN_EN_PIN, HIGH);
	else digitalWrite(m_PWN_EN_PIN, LOW);
}*/

void SB_Motor_L298N::SetTicksPerRev(float ticksPerRev) {m_ticksPerRev = ticksPerRev;}

void SB_Motor_L298N::SetKp(float Kp) {
	m_Kp = Kp;
	myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}
void SB_Motor_L298N::SetKi(float Ki) {
	m_Ki = Ki;
	myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}
void SB_Motor_L298N::SetKd(float Kd) {
	m_Kd = Kd;
	myPID.SetTunings(m_Kp, m_Ki, m_Kd);
}
void SB_Motor_L298N::SetGoalPos(float Setpoint) { *m_Setpoint = Setpoint; }

void SB_Motor_L298N::Off() {
	pwmValN = 0;
  	pwmValP = 0;
	digitalWrite(m_L298N_IN1, pwmValP);
	digitalWrite(m_L298N_IN2, pwmValN);
}

float SB_Motor_L298N::GetKp() {return m_Kp;}
float SB_Motor_L298N::GetKi() {return m_Ki;}
float SB_Motor_L298N::GetKd() {return m_Kd;}
float SB_Motor_L298N::GetGoalPos() {return *m_Setpoint;}
float SB_Motor_L298N::GetTicksPerRev() {return m_ticksPerRev;}
bool SB_Motor_L298N::GetInit() {return m_init;}
bool SB_Motor_L298N::GetStop() {return m_stop;}

