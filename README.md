# Roller Grasper V1 Reproduction

This repository contains the reproduction for the design in the paper titled "Design of a Roller-Based Dexterous Hand for Object Grasping and Within-Hand Manipulation." This fantastic robotic hand is designed by Shenli Yuan. I like the design at my first sight when reading his paper. Therefore, I reproduced this robotic hand based on the implementations in his paper and some helperful advice from him.

## Description
### Control program
The \Code folder includes the control program folder and library folder.
* Current based position control for dynamixels
* Arduino Servo library for pivot servo motors
* PD controller for roller motors
* Custom serial communication protocol built on top of the protocol developed by [Rohan Agrawal](https://github.com/rohbotics?tab=repositories)

### Testing Video
The grasping performance is tested on five different objects, including bottle, box, die, paper, and sphere.

### Summary of the implementation
The .pdf file includes the summary of the paper when I read it. Based on the kinematic analysis and implementation instructions in the paper, I reproduced the design.

## Electronics
The circuit diagram can be found in the paper. 

[Teensy 3.6](http://www.robotis.us/dynamixel-xh430-w350-t/): handles serial communication and low level controllers

[SMPS2Dynamixel Adapter](https://www.trossenrobotics.com/store/p/5886-SMPS2Dynamixel-Adapter.aspx): used as power regulator for the whole circuit

[Dynamixel DYNAMIXEL XM430-W350-T](http://www.robotis.us/dynamixel-xh430-w350-t/): base joints

[Savox SW-0250MG Servo](https://www.savox-servo.com/home.php): pivot joints

[Micro gear motors](https://www.servocity.com/110-rpm-micro-gear-motor-w-encoder): roller joints

[L298N Motor Driver Carrier](https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/): dirve the Micro gear motors

## API example
- under control program, run the five different .ino file in Arduino IDE, the robotic hand will begin grasping movement after input '1' in the serial monitor.

- under control program, run the last .ino file in Arduino IDE, input the positions(angle) for each motor in the serial monitor, the hand will operate as expected.

## Reference
```
@INPROCEEDINGS{9197146,  
author={Yuan, Shenli and Epps, Austin D. and Nowak, Jerome B. and Salisbury, J. Kenneth},  
booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},   
title={Design of a Roller-Based Dexterous Hand for Object Grasping and Within-Hand Manipulation},   
year={2020},  volume={},  number={},  pages={8870-8876},
doi={10.1109/ICRA40945.2020.9197146}}
```
