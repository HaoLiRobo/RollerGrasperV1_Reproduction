# Roller Grasper V1 Reproduction

This repository contains the reproduction for the design in the paper titled "Design of a Roller-Based Dexterous Hand for Object Grasping and Within-Hand Manipulation." This fantastic robotic hand is designed by Shenli Yuan. I like the design at my first sight when reading his paper. Therefore, I reproduced this robotic hand based on the implementations in his paper and some helperful advice from him.

## Description


## Electronics
The circuit diagram can be found in the paper. 

[Teensy 3.6](http://www.robotis.us/dynamixel-xh430-w350-t/): handles serial communication and low level controllers

[SMPS2Dynamixel Adapter](https://www.trossenrobotics.com/store/p/5886-SMPS2Dynamixel-Adapter.aspx): used as power regulator for the whole circuit

[Dynamixel DYNAMIXEL XM430-W350-T](http://www.robotis.us/dynamixel-xh430-w350-t/): base joints

[Micro gear motors](https://www.servocity.com/110-rpm-micro-gear-motor-w-encoder): pivot joints and roller joints

[L298N Motor Driver Carrier]: dirve the Micro gear motors

## Demos

## Reference
```
@INPROCEEDINGS{9197146,  
author={Yuan, Shenli and Epps, Austin D. and Nowak, Jerome B. and Salisbury, J. Kenneth},  
booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},   
title={Design of a Roller-Based Dexterous Hand for Object Grasping and Within-Hand Manipulation},   
year={2020},  volume={},  number={},  pages={8870-8876},
doi={10.1109/ICRA40945.2020.9197146}}
```
