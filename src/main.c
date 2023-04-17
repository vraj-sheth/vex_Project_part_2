/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "pros/adi.h"
#include "pros/motors.h"
#include "main.h"
#include "pros/llemu.h"
#include <stdint.h>
#include "Student_Code.h"

/* Background processing variables. Do not modify or delete */
task_t monitorMotors_Task;
task_t check_sensors;
int _stopflag = 0;
int _arm_State = 0;

/* Runs initialization code. This occurs as soon as the program is started. Do not touch */
void initialize() {
	
	//initialise LCD screen
	lcd_initialize();
	
	//begin background processing tasks
	delay(200);
	check_sensors = task_create(checkSensors, NULL, TASK_PRIORITY_DEFAULT+2, TASK_STACK_DEPTH_DEFAULT, "Check sensors");
	delay(200);
	monitorMotors_Task = task_create(monitorMotorPower, NULL, TASK_PRIORITY_DEFAULT+1, TASK_STACK_DEPTH_DEFAULT, "Monitor Motor Power");
	delay(200);

	//initialise and configure adi pins
	int32_t success = adi_port_set_config(_lightLeft, E_ADI_ANALOG_IN);
	success = adi_port_set_config(_lightMid, E_ADI_ANALOG_IN);
	success = adi_port_set_config(_lightRight, E_ADI_ANALOG_IN);
	success = adi_port_set_config(_armLimitLow, E_ADI_DIGITAL_IN);
	success = adi_port_set_config(_armLimitHigh, E_ADI_DIGITAL_IN);
	success = adi_port_set_config(_buttonStop, E_ADI_DIGITAL_IN);

	//initialise left wheel motor
  	motor_set_gearing(_motorLeft, E_MOTOR_GEARSET_18);
  	motor_set_reversed(_motorLeft, false);
  	motor_set_encoder_units(_motorLeft, E_MOTOR_ENCODER_COUNTS);

	//initialise right wheel motor (reversed)
	motor_set_gearing(_motorRight, E_MOTOR_GEARSET_18);
  	motor_set_reversed(_motorRight, true);
  	motor_set_encoder_units(_motorRight, E_MOTOR_ENCODER_COUNTS);

	//initialise robot arm motor
	motor_set_gearing(_motorArm, E_MOTOR_GEARSET_18);
  	motor_set_reversed(_motorArm, false);
  	motor_set_encoder_units(_motorArm, E_MOTOR_ENCODER_COUNTS);
	motor_set_brake_mode(_motorArm, 2);

	//short delay before student's code - DO NOT REMOVE!
	delay(100);

	//run student's code (RUNNING AS TASK NEEDS TESTING)
	student_Main();
	endOfProgram();
	// student_Task = task_create(student_task, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Student main task");
}