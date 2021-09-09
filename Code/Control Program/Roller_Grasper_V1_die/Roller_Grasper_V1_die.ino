#include <Arduino.h>
#include "sb_dynamixel.h"
#include "sb_motor_l298n.h"
#include "sb_servo.h"
#include <algorithm>
#include <iterator>
 
#define TickPerRev 3700
#define Deg2Tick (float)TickPerRev/360.0

#define DEBUG_SERIAL Serial
//#define InputSerial Serial2
#define ANALOG_RES 12
#define BIT2DEG  360.0/pow(2.0, (float)ANALOG_RES)
#define DEG2BIT  pow(2.0, (float)ANALOG_RES)/360.0

int paramSequence[8][8] = {{0,0,0,0,0,88,84,88}, {180,0,0,0,0,88,84,88},{180,0,0,1,0,88,84,88},{110,70,70,0,0,88,84,88},{40,140,140,0,0,88,84,88,},{-20,200,200,0,0,88,84,88,},{-20,200,200,0,0,88,110,88},{-20,200,200,0,1,88,110,88}};

int curr_step = 0;

/* Servo Motors */
SB_SERVO servos[3]= {
  SB_SERVO(0,SERVO_POS,120,10,18),
  SB_SERVO(1,SERVO_POS,120,10,12),
  SB_SERVO(2,SERVO_POS,160,50,153),
};

/* DC Motors */
SB_Motor_L298N motors[3] = {
  SB_Motor_L298N(0,3700, 7.2,0, 0.3,Input, Output, Setpoint),
  SB_Motor_L298N(1,3700, 7.2,0, 0.3,Input, Output, Setpoint),
  SB_Motor_L298N(2,3700, 7.2,0, 0.3,Input, Output, Setpoint),
  
};

/* Dynamixels */
SB_Dynamixel baseMotor[3] = {
  SB_Dynamixel(100, 0, 0, 0,0),
  SB_Dynamixel(101, 0, 0, 0,0),
  SB_Dynamixel(102, 0, 0, 0,0),
};

int jointInputs[4][3] = {{200, 200, 200},     // dynamixel current
  {0, 0, 0},     // dynamixel pos
  {0, 0, 0},     // pivot pos (not used in this code)
  {0, 0, 0}     // roller pos (not used in this code)
};

const int posLowLimDeg = 80;
const int posHighLimDeg = posLowLimDeg + 90;
const int posLowLimBits = posLowLimDeg * DEG2BIT;
const int posHighLimBits = posHighLimDeg * DEG2BIT;
const int currentLowLimBits = 0;
const int currentHighLimBits = 1193;
String comdata = "";
int numdata[8] = {0}, mark = 0;

bool servo_move=false;
bool two_servo_move= false;

uint32_t startTime =millis();
int myCounter = 0;
String joint_value;

bool minus =false;

String Setpoint_Value;
int Val=0;

void setup() {
  // put your setup code here, to run once:
  INI_POS[0] = 120;//120
  INI_POS[1] = 12;//12
  INI_POS[2] = 50;
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.print("Ready");
  analogReadResolution(ANALOG_RES);
  delay(2000);
  
  for(int i=0;i<3;i++)
  {
    motors[i].Init(TickPerRev,7.2,0,0.3);
    baseMotor[i].init();
    servos[i].Init(MAX_POS[i],MIN_POS[i],INI_POS[i]);
    
  }

    jointInputs[1][0]= 88 * DEG2BIT;//83
    jointInputs[1][1]= 84* DEG2BIT;//85
    jointInputs[1][2]= 88* DEG2BIT;//90
  
  delay(1000);  
  pinMode(LED_BUILTIN, OUTPUT);
  blinkNTimes(5);
  startTime =millis();
  //Serial.print(startTime);
  }

void loop() {


      while(Serial.available()>0)          //检测串口是否接收到了数据
  {
    Setpoint_Value=Serial.readString();  //读取串口字符串
    Val = Setpoint_Value.toInt();
    Serial.print("开始运行");//串口监视器输入“1”以开始运行程序
  }
      if(Val==1)
      {
           if(millis()-startTime>=3000 && curr_step<8)
   {
   startTime = millis();
   Setpara(paramSequence[curr_step][0], 
   paramSequence[curr_step][1], 
   paramSequence[curr_step][2], 
   paramSequence[curr_step][3], 
   paramSequence[curr_step][4], 
   paramSequence[curr_step][5], 
   paramSequence[curr_step][6], 
   paramSequence[curr_step][7]); 
   curr_step += 1;
   }  
      }

    
    
    updateDynamixels(false);
    motors[0].Update();
    motors[1].Update();
    motors[2].Update();
   
    Two_Servo_update();
    middle_servo_update();
}

void updateDynamixels(bool ifPrint) {
  for (int i = 0; i < 3; i++) {
    baseMotor[i].set_goal_current(map(jointInputs[0][i], 1, 4095, 0, 1193));
    baseMotor[i].current_goal();
    jointInputs[1][i] = min(jointInputs[1][i], posHighLimBits); // set pos
    jointInputs[1][i] = max(jointInputs[1][i], posLowLimBits);  // go to pos
    //DEBUG_SERIAL.println(jointInputs[1][i]);
    baseMotor[i].set_goal_position(jointInputs[1][i]);
    baseMotor[i].position_goal();
    if (ifPrint) {
      DEBUG_SERIAL.print(baseMotor[i].get_goal_current());
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print(baseMotor[i].get_goal_position());
      DEBUG_SERIAL.print(", ");
    }
  }
  if (ifPrint) DEBUG_SERIAL.println();
}

void middle_servo_update(){
  if(servo_move==true)
  {
    for (int pos = 0; pos < 105; pos += 1)     
      {
        servos[1].SetGoalPos(servos[1].m_last_pos+1);
        servos[1].Update();
                                            
    }
    servo_move=false;

}
}

void Two_Servo_update()
{
 if(two_servo_move==true)
  {

     for (int pos = 0; pos < 105; pos += 1)     
      {
        servos[0].SetGoalPos(servos[0].m_last_pos-1);
        servos[0].Update();
        servos[2].SetGoalPos(servos[2].m_last_pos+1);
        servos[2].Update();
                                            
    }
    two_servo_move=false;
  }
  
}


void Middle_motor_move(int deg)
{
  if(motors[0].m_lastSetpoint < deg*Deg2Tick)
  {
    for (; motors[0].m_lastSetpoint < deg*Deg2Tick;)     
      {
        motors[0].SetGoalPos(motors[0].m_lastSetpoint+Deg2Tick);
        Setpoint[0]= (motors[0].m_lastSetpoint+Deg2Tick);
        delay(20);
                      
    }
  }
  else
  {
        for (; motors[0].m_lastSetpoint > deg*Deg2Tick;)     
      {
        motors[0].SetGoalPos(motors[0].m_lastSetpoint+Deg2Tick);
        Setpoint[0]= (motors[0].m_lastSetpoint-Deg2Tick);
        delay(20);
                      
    }
  }

}


void Setpara(int num_1, int num_2, int num_3,int num_4,int num_5,int num_6,int num_7,int num_8)
{
  
        Setpoint[0]=num_1*Deg2Tick;
        Setpoint[1]=num_2*Deg2Tick;
        Setpoint[2]=num_3*Deg2Tick;

        servo_move =num_4;
        two_servo_move =num_5;


        jointInputs[1][0]= num_6 * DEG2BIT;
        jointInputs[1][1]= num_7* DEG2BIT;
        jointInputs[1][2]= num_8* DEG2BIT;
}


void blinkNTimes(uint8_t num) {
  for (int i = 0; i < num; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  delay(500);
}
