
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "sb_servo.h"


const int SERVO_PWM[3] = {4, 5, 6};
const int MAX_POS[3] = {120, 120,160};
const int MIN_POS[3] = {10,10,50};
int INI_POS[3] = {115,12,48};
 
int SERVO_POS[3] = {0,0,0};

SB_SERVO::SB_SERVO(int servoID, int* servo_goal_pos, int max_pos, int min_pos, int ini_pos):
	myServo(){

	m_servoID = servoID;
	m_servo_goal_pos = servo_goal_pos;

	m_max_pos = max_pos;
	m_min_pos = min_pos;
	m_ini_pos = ini_pos;
	m_last_pos = ini_pos;

	m_SERVO_PWM = SERVO_PWM[m_servoID];
	}

void SB_SERVO::Init(int max_pos, int min_pos, int ini_pos) {
	m_max_pos = max_pos;
	m_min_pos = min_pos;
	m_ini_pos = ini_pos;
	m_last_pos = ini_pos;
	

	switch (m_servoID) {	
	case 0:
		myServo.attach(SERVO_PWM[0]);
		break;
	case 1:
		myServo.attach(SERVO_PWM[1]);
		break;
	case 2:
		myServo.attach(SERVO_PWM[2]);
		break;
	default:
		Serial.println("init default");
		break;
	}
	myServo.write(m_ini_pos);
	delay(1000);
	m_init = true;

}

void SB_SERVO::Update() {
	if (*m_servo_goal_pos >= m_min_pos && *m_servo_goal_pos <= m_max_pos)
	{

		myServo.write(*m_servo_goal_pos); 
		delay(15);                       
		/*if (m_last_pos < *m_servo_goal_pos)
		{
			myServo.write(*m_servo_goal_pos);              // tell servo to go to position in variable 'pos' 
			delay(15);                       // waits 15ms for the servo to reach the position
			for (int pos = m_last_pos; pos <= *m_servo_goal_pos; pos += 1)     // goes from 180 degrees to 0 degrees 
			{
				myServo.write(pos);              // tell servo to go to position in variable 'pos' 
				delay(15);                       // waits 15ms for the servo to reach the position 
			}
			//delay(1000);
		}
		else
		{
			
			for (int pos = m_last_pos; pos >= *m_servo_goal_pos; pos -= 1)     // goes from 180 degrees to 0 degrees 
			{
				myServo.write(pos);              // tell servo to go to position in variable 'pos' 
				delay(15);                       // waits 15ms for the servo to reach the position 
			}
			//delay(1000);
		}*/

		m_last_pos = *m_servo_goal_pos;
	}

}

void SB_SERVO::SetGoalPos(int goal_pos) { *m_servo_goal_pos = goal_pos; }

int SB_SERVO::GetLastPos() { return m_last_pos;}
int SB_SERVO::GetGoalPos() { return *m_servo_goal_pos; }
int SB_SERVO::GetServoID() { return m_servoID; }
int SB_SERVO::GetMaxPos() { return m_max_pos; }
int SB_SERVO::GetMinPos() { return m_min_pos; }
int SB_SERVO::GetIniPos() { return m_ini_pos; }
bool SB_SERVO::GetInit() { return m_init; }




