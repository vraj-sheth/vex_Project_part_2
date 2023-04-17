/**
 * @file Background_Tasks.c
 * @author Hazim Namik
 * @brief This file contains definitions for tasks that run the background. These are meant to protect the robot from accidental misuse (programming errors).
 * @version 0.1
 * @date 2023-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pros/adi.h"
#include "pros/llemu.h"
#include "pros/misc.h"
#include "pros/motors.h"

// adi_ultrasonic_t sonar;

/// @brief This taks regularly checks the power being sent to the motors. If power is being sent but the motors are not moving, it will stop all the motors and prevent further motor activation.
/// @param none
void monitorMotorPower(void *param)
{
	int threshold = 500;	// voltage threshold in mV to ignore in power variations (it's 0.5 V to allow for the arm motor to be in hold mode)
	int powerLMonitor, powerRMonitor, powerArmMonitor;
	int countL, countR, countA;

	while (1)
	{		
		// GET THE MOTOR POWER LEVELS.
		powerLMonitor = motor_get_voltage (_motorLeft);
		powerRMonitor = motor_get_voltage (_motorRight);
		powerArmMonitor = motor_get_voltage (_motorArm);
	
		countL = motor_get_position(_encoderLeft);
		countR = motor_get_position(_encoderRight);
		countA = motor_get_position(_encoderArm);

		// lcd_print(LCDLine7,"Arm = %d",powerArmMonitor);
		// DETERMINE CHANGE IN ENCODER VALUES AFTER 5 SECONDS.
		task_delay(10000); 

		// Check left motor
		if ((abs(powerLMonitor) > threshold)  && ((motor_get_position(_encoderLeft) - countL) == 0) )
		{
			motorStopAll();
			_stopflag = 1;
			lcd_print(LCDLine8, "%dmV-Low L motor pwr. Code stopped.", powerLMonitor);
		}
		// Check right motor
		if ((abs(powerRMonitor) > threshold)  && ((motor_get_position(_encoderRight) - countR) == 0) )
		{
			motorStopAll();
			_stopflag = 1;
			lcd_print(LCDLine8, "%dmV-Low R motor pwr. Code stopped.", powerRMonitor);
		}
		// Check arm motor
		if ((abs(powerArmMonitor)> threshold) && ((motor_get_position(_encoderArm) - countA) == 0) )
		{
			motorStopAll();
			_stopflag = 1;
			lcd_print(LCDLine8, "%dmV-Low arm motor pwr. Code stopped.", powerArmMonitor);
		}

	}
}


/// @brief This taks regularly checks the stop button and arm limit switches.
/// If the stop buttons is pressed, it will stop all the motors and prevent further motor activation.
/// When any of the arm limit switches are triggered, it will stop the arm motor from driving in the direction that triggered the
/// limit switch (e.g., stop raising the arm when the upper limit swith is triggered.)
/// @param none
void checkSensors(void *param)
{
	bool stopButton = adi_digital_read(_buttonStop);
	int arm_motorDir;
	task_delay(500); // this delay is necessary because for some reason it initialises with the Stop Button pressed. 
	lcd_print(LCDLine8, "    Running ...    ");
	while (1)
	{

		// Checking the Stop Button
		stopButton = adi_digital_read(_buttonStop);
		if (stopButton == 1)
		{
			lcd_print(LCDLine8, "    STOP BUTTON PRESSED    ");
			motorStopAll();
			_stopflag = 1;
		}

		// Checking the arm limit switches
		arm_motorDir = motor_get_direction(_motorArm); // Check what the motor is doing (1 is raising, -1 is lowering, 0 is stopped)
		if (adi_digital_read(_armLimitLow))		   // Check lower limit switch
		{
			_arm_State = -1; 					// update arm_State (-1 = at lower limit)
			if (arm_motorDir < 0)
			{
				motor_move(_motorArm, 0); 		// stop if commaded to lower the arm
			}
		}
		else
		{
			if (adi_digital_read(_armLimitHigh))
			{
				_arm_State = 1; 				// update arm_State (1 = at upper limit)
				if (arm_motorDir > 0)
				{
					motor_move(_motorArm, 0); 	// stop if commaded to raise the arm
				}
			} else{ 							// arm is in between the limit switches
				_arm_State = 0; 				// update arm_State (0 = somewhere between the limits)
			}
		}

		// DELAY 20MS
		task_delay(20);
	}
}

void motorStopAll()
{
	motor_move(_motorLeft, 0);
	motor_move(_motorRight, 0);
	motor_move(_motorArm, 0);
}