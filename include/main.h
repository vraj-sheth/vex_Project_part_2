/**
 * @file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_
#define PROS_USE_LITERALS

#include "api.h"

/* Pin allocations for motors and sensors */
#define _encoderLeft    18  //left wheel encoder
#define _motorLeft      18  //left wheel motor
#define _encoderRight   19  //right wheel encoder
#define _motorRight     19  //right wheel motor
#define _encoderArm     20  //robot arm encoder
#define _motorArm       20  //robot arm motor
#define _sonarPing      'A' //sonar ping
#define _sonarEcho      'B' //sonar echo
#define _lightRight     'C' //right light sensor
#define _lightMid       'D' //middle light sensor
#define _lightLeft      'E' //left light sensor
#define _armLimitLow    'F' //lower limit switch of robot arm
#define _armLimitHigh   'G' //upper limit switch of robot arm
#define _buttonStop     'H' //stop button

/* Motor macros for student use */
#define RightMotor      0   //right wheel motor
#define LeftMotor       1   //left wheel motor
#define ArmMotor        2   //robot arm motor

/* Sensor macros for student use */
#define MidLight        0   //middle light sensor
#define LeftLight       1   //left light sensor
#define RightLight      2   //right light sensor
//#define StartButton     3   //(NOT IMPLEMENTED) start button
#define StopButton      4   //stop button
#define RightEncoder    5   //right wheel encoder
#define LeftEncoder     6   //left wheel encoder
#define LowArmLimit     7   //lower limit switch of robot arm
#define HighArmLimit    8   //upper limit switch of robot arm
#define ArmEncoder      9   //robot arm encoder
#define SonarSensor     10  //sonar sensor

/* LCD macros for student use */
#define LCDLine1        0   //line 1 of LCD (SHOWS BATTERY LEVEL)
#define LCDLine2        1   //line 2 of LCD
#define LCDLine3        2   //line 3 of LCD
#define LCDLine4        3   //line 4 of LCD
#define LCDLine5        4   //line 5 of LCD
#define LCDLine6        5   //line 6 of LCD
#define LCDLine7        6   //line 7 of LCD
#define LCDLine8        7   //line 8 of LCD

/* Timer initialisation and macros */
extern long T1_timer;
extern long T2_timer;
extern long T3_timer;
extern long T4_timer;
#define T_1             0   //timer #1
#define T_2             1   //timer #2
#define T_3             2   //timer #3
#define T_4             3   //timer #4

/* Macros for universal motor limits */
#define MOTOR_FLOOR     -5000
#define MOTOR_CEILING   5000
#define PI 3.14159265359

void initialize(void);

/* Function declarations from Background_Functions.c */

double saturate(double input, double lower, double upper);
int getMotorPower(int motorName);
int readSensor (int sensorName);
int readTimer(int timerSelect);
void armDown(int voltage);
void armUp(int voltage);
void motorPower(int motorName, int voltage);
void resetTimer(int timerSelect);
void resetEncoder(int encoderName);
void endOfProgram();
double min(double num1, double num2);
double max(double num1, double num2);
int sgn(double input);

/* Function declarations from Background_Tasks.c */

void monitorMotorPower(void* param);
void motorStopAll(void);
void checkSensors(void* param);

/* Global variables */
extern int _stopflag;           //1 || 0 - Used to control whether or not robot is to be stopped or in stop mode.
extern int _arm_State;          // Variable that defines the state of the robot arm (-1 at lower limit, 1 at upper limit, and 0 in between)

#endif  // _PROS_MAIN_H_
