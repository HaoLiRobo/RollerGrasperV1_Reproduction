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
  SB_Dynamixel(100, 0, 0, 0, 0),
  SB_Dynamixel(101, 0, 0, 0, 0),
  SB_Dynamixel(102, 0, 0, 0, 0),
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

uint32_t startTime = millis();
int myCounter = 0;
String joint_value;

bool minus =false;

int distance[3] = {0};
int max_dis = 0;
int max_index = 0;


void setup() {
  // put your setup code here, to run once:
  INI_POS[0] = 120;//120
  INI_POS[1] = 12;//12
  INI_POS[2] = 50;//50
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

    jointInputs[1][0]= 105 * DEG2BIT;
    jointInputs[1][1]= 110* DEG2BIT;
    jointInputs[1][2]= 105* DEG2BIT;
  
  delay(1000);  
  pinMode(LED_BUILTIN, OUTPUT);
  blinkNTimes(5);
  }

void loop() {

      int j = 0;
     while (Serial.available() > 0)//串口监视器输入每个joint的值，例如：90，90，90，0，0，100，100，100
      {
        comdata += char(Serial.read());
        delay(2);
        mark = 1;
      }
     
      if(mark == 1)
      {
        Serial.println(comdata);
        Serial.println(comdata.length());
        for(int i = 0; i < comdata.length() ; i++)
        {
          if(comdata[i] == ',')
          {
            
            if(minus ==true){
              numdata[j] = -numdata[j];
              minus = false;
            }
            
            j++;
          }
          else
          {
            if(comdata[i] =='-')
          {
             minus = true;
             i = i+1;
                          
          }
            
            numdata[j] = numdata[j] * 10 + (comdata[i] - '0');
          
        }
        }
        comdata = String("");

        Setpoint[0]=numdata[0]*Deg2Tick;
        Setpoint[1]=numdata[1]*Deg2Tick;
        Setpoint[2]=numdata[2]*Deg2Tick;

        servo_move =numdata[3];
        two_servo_move =numdata[4];


        jointInputs[1][0]= numdata[5] * DEG2BIT;
        jointInputs[1][1]= numdata[6]* DEG2BIT;
        jointInputs[1][2]= numdata[7]* DEG2BIT;
               

        for(int i = 0; i <8; i++)
        { 
          Serial.print(numdata[i]);
          Serial.print(',');
          numdata[i] = 0;
        }
        mark = 0;
      }


    
    motors[0].Update();
    motors[1].Update();
    motors[2].Update();
    
    updateDynamixels(false);

   
    Two_Servo_update();
    middle_servo_update();

}


void updateDynamixels(bool ifPrint) {

  
  for (int i = 0; i < 3; i++) {
    baseMotor[i].set_goal_current(map(jointInputs[0][i], 1, 4095, 0, 1193));
    baseMotor[i].current_goal();
    jointInputs[1][i] = min(jointInputs[1][i], posHighLimBits); // set pos
    jointInputs[1][i] = max(jointInputs[1][i], posLowLimBits);  // go to pos
    distance[i] =abs(baseMotor[i].m_last_position-jointInputs[1][i]);
  }

  for(int i = 0; i < 3; i++)
  {
    if(distance[i] > max_dis) {
        max_dis = distance[i];
        max_index = i;
    }
  }
    
   for(;abs(baseMotor[max_index].m_last_position-jointInputs[1][max_index])>0;)
    {
          //DEBUG_SERIAL.println(jointInputs[1][i]);
          if(baseMotor[0].m_last_position-jointInputs[1][0]<0)
          {
           baseMotor[0].set_goal_position(baseMotor[0].m_last_position+1);
          }
          else if(baseMotor[0].m_last_position-jointInputs[1][0]>0)
          {
            baseMotor[0].set_goal_position(baseMotor[0].m_last_position-1);
          }
          if(baseMotor[1].m_last_position-jointInputs[1][1]<0)
          {
           baseMotor[1].set_goal_position(baseMotor[1].m_last_position+1);
          }
          else if(baseMotor[1].m_last_position-jointInputs[1][1]>0)
          {
            baseMotor[1].set_goal_position(baseMotor[1].m_last_position-1);
          }
          if(baseMotor[2].m_last_position-jointInputs[1][2]<0)
          {
           baseMotor[2].set_goal_position(baseMotor[2].m_last_position+1);
          }
          else if(baseMotor[2].m_last_position-jointInputs[1][2]>0)
          {
            baseMotor[2].set_goal_position(baseMotor[2].m_last_position-1);
          }     
           baseMotor[0].position_goal();
           baseMotor[1].position_goal();
           baseMotor[2].position_goal();
           delay(20);

    } 
}

/*void updateDynamixels(bool ifPrint) {
  for (int i = 0; i < 3; i++) {
    baseMotor[i].set_goal_current(map(jointInputs[0][i], 1, 4095, 0, 1193));
    baseMotor[i].current_goal();
    jointInputs[1][i] = min(jointInputs[1][i], posHighLimBits); // set pos
    jointInputs[1][i] = max(jointInputs[1][i], posLowLimBits);  // go to pos
    
    if(baseMotor[i].m_last_position<jointInputs[1][i])
    {
          for(;baseMotor[i].m_last_position<jointInputs[1][i];)
    {
          //DEBUG_SERIAL.println(jointInputs[1][i]);
    baseMotor[i].set_goal_position(baseMotor[i].m_last_position+1);
    baseMotor[i].position_goal();
    delay(10);
    }
    }
    else if(baseMotor[i].m_last_position>jointInputs[1][i])
    {
             for(;baseMotor[i].m_last_position>jointInputs[1][i];)
    { 
          //DEBUG_SERIAL.println(jointInputs[1][i]);
    baseMotor[i].set_goal_position(baseMotor[i].m_last_position-1);
    baseMotor[i].position_goal();
    delay(10);
    }
    }
   
    if (ifPrint) {
      DEBUG_SERIAL.print(baseMotor[i].get_goal_current());
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print(baseMotor[i].get_goal_position());
      DEBUG_SERIAL.print(", ");
    }
  }
  if (ifPrint) DEBUG_SERIAL.println();
}*/


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



/*void Middle_motor_move(int deg)
{
    Setpoint[0]=deg*Deg2Tick;
}*/


void Two_motor_move(int deg)
{

    Setpoint[1]=deg*Deg2Tick;
    Setpoint[2]=deg*Deg2Tick;

  
}

void Three_motor_move(int deg)
{
    Setpoint[0]=(deg-90)*Deg2Tick;
    Setpoint[1]=deg*Deg2Tick;
    Setpoint[2]=deg*Deg2Tick;

  
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
