#ifndef SB_SERVO_H
#define SB_SERVO_H


#include "Servo.h"

extern const int SERVO_PWM[3];
extern const int MAX_POS[3];
extern const int MIN_POS[3];
extern int INI_POS[3];

extern int SERVO_POS[3];



class SB_SERVO
{
public:


	SB_SERVO(int servoID, int* servo_goal_pos, int max_pos, int min_pos, int ini_pos);
	void Init(int max_pos, int min_pos, int ini_pos);

	void SetGoalPos(int goal_pos);
	int GetLastPos();
	int GetGoalPos();
	int GetServoID();
	int GetMaxPos();
	int GetMinPos();
	int GetIniPos();
	bool GetInit();
	void Update();

	int m_last_pos;
	int* m_servo_goal_pos =0;

private:
	int m_servoID;
	bool m_init = false;
	int m_max_pos;
	int m_min_pos;
	int m_ini_pos;


	int m_SERVO_PWM;

	Servo myServo;
};



#endif
