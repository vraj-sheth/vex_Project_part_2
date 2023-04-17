/**
 * @file Background_Functions.c
 * @author Hazim Namik
 * @brief This file provides functions that will make controlling the VEX robot easier across different platforms (e.g., in the lab and using the simulator).
 * @version 0.1
 * @date 2023-02-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "main.h"
#include "pros/adi.h"

adi_ultrasonic_t sonar;
bool Ultra_Init = false;
long T1_timer = 0;
long T2_timer = 0;
long T3_timer = 0;
long T4_timer = 0;

// __[ GET MOTOR POWER ]________________________________________________
/**
 * @brief Reads the voltage being sent to the motor.
 * @param motorName Options include: LeftMotor, RightMotor, or ArmMotor
 * @return motor voltage in mV as an integer.
 */
int getMotorPower(int motorName)
{

	int motorOutput;

	// select motor to read
	switch (motorName)
	{
	case 0: // read right wheel motor power
		motorOutput = motor_get_voltage(_motorRight);
		break;
	case 1: // read left wheel motor power
		motorOutput = motor_get_voltage(_motorLeft);
		break;
	case 2: // read robot arm motor power
		motorOutput = motor_get_voltage(_motorArm);
		break;
	}
	return motorOutput;
}

// __[ READ SENSOR ]____________________________________________________
/**
 * @brief Reads the output of the selected sensor.
 * @param sensorName Desired sensor. Options include: LeftEncoder, RightEncoder, ArmEncoder, LeftLight, MidLight, RightLight, SonarSensor, StopButton, LowArmLimit, HighArmLimit.
 * @return iOutput of the selected sensor as an integer.
 */
int readSensor(int sensorName)
{

	int sensorOutput, currentSonar_cm, currentSonar_mm, prevSonar;

	// initialise sonar sensor if not already being read
	if (!Ultra_Init)
	{
		sonar = adi_ultrasonic_init(_sonarPing, _sonarEcho);
		Ultra_Init = true;
		delay(100);	// required to give sensor time to initialise and stabilise.
	}

	// select sensor to read
	switch (sensorName)
	{
	case 0: // middle light sensor
		sensorOutput = adi_analog_read(_lightMid);
		break;
	case 1: // left light sensor
		sensorOutput = adi_analog_read(_lightLeft);
		break;
	case 2: // right light sensor
		sensorOutput = adi_analog_read(_lightRight);
		break;
	case 4: // stop button
		sensorOutput = adi_digital_read(_buttonStop);
		break;
	case 5: // right wheel encoder
		sensorOutput = motor_get_position(_encoderRight);
		break;
	case 6: // left wheel encoder
		sensorOutput = motor_get_position(_encoderLeft);
		break;
	case 7: // lower limit switch of robot arm
		sensorOutput = adi_digital_read(_armLimitLow);
		break;
	case 8: // upper limit switch of robot arm
		sensorOutput = adi_digital_read(_armLimitHigh);
		break;
	case 9: // robot arm encoder
		sensorOutput = motor_get_position(_motorArm);
		break;
	case 10: // sonar sensor
		currentSonar_cm = adi_ultrasonic_get(sonar);
		currentSonar_mm = currentSonar_cm * 1; // sonar is actually reading in mm!
		if (prevSonar == 0 && currentSonar_mm == 0)
		{
			sensorOutput = -1;
		}
		else if (prevSonar != 0 && currentSonar_mm == 0)
		{
			sensorOutput = prevSonar;
			prevSonar = 0;
		}
		else
		{
			sensorOutput = currentSonar_mm;
			prevSonar = sensorOutput;
		}
		break;
	}
	return sensorOutput;
}

// __[ READ TIMER ]_____________________________________________________
/**
 * @brief Reads the elapsed time of a selected timer.
 * @param timerSelect Available timers: T_1, T_2, T_3, or T_4
 * @return int Elapsed time of timer in milliseconds.
 */
int readTimer(int timerSelect)
{

	int timerOutput;

	// select timer to read
	switch (timerSelect)
	{
	case 0: // timer #1
		timerOutput = millis() - T1_timer;
		break;
	case 1: // timer #2
		timerOutput = millis() - T2_timer;
		break;
	case 2: // timer #3
		timerOutput = millis() - T3_timer;
		break;
	case 3: // timer #4
		timerOutput = millis() - T4_timer;
		break;
	}
	return timerOutput;
}

// __[ ARM DOWN ]_______________________________________________________
/**
 * @brief Raises the arm until the upper lower switch is activated. CANNOT be stopped before it reaches the bottom position.
 * @param voltage Integer value for the desired motor voltage in mV.
 */
void armDown(int voltage)
{

	int armMin = adi_digital_read(_armLimitLow);

	// constrain motor power to safe operating range
	int powerOutput = (-1) * saturate(abs(voltage), 0, MOTOR_CEILING);

	// lower arm until lower limit switch is reached
	while (armMin == 0)
	{
		motorPower(ArmMotor, powerOutput);
		armMin = adi_digital_read(_armLimitLow);
		delay(20);
	}

	// stop arm motor
	motorPower(ArmMotor, 0);
}

// __[ ARM UP ]_________________________________________________________
/**
 * @brief Raises the arm until the upper limit switch is activated. CANNOT be stopped before it reaches the top position.
 * @param voltage Integer value for the desired motor voltage in mV.
 */
void armUp(int voltage)
{

	int armMax = adi_digital_read(_armLimitHigh);

	// constrain motor power to safe operating range
	int powerOutput = (1) * saturate(abs(voltage), 0, MOTOR_CEILING);

	// raise arm until upper limit switch is reached
	while (armMax == 0)
	{
		motorPower(ArmMotor, powerOutput);
		armMax = adi_digital_read(_armLimitHigh);
		delay(20);
	}

	// stop arm motor
	motorPower(ArmMotor, 0);
}

//------------------------------------ motorPower --------------------------------------
/**
 * @brief Sets the power of a selected motor to a desired level.
 * @param motorName LeftMotor, RightMotor, or ArmMotor
 * @param voltage Integer value for the desired motor voltage in mV.
 */
void motorPower(int motorName, int voltage)
{

	if (_stopflag) // if stop button has been pressed, don't allow any motorPower command to send power to any motor
	{
		voltage = 0;
	}

	// constrain motor power to safe operating range
	int powerOutput = (int)saturate((double)voltage, MOTOR_FLOOR, MOTOR_CEILING);

	// select motor for actuation
	switch (motorName)
	{
	case 0: // right wheel motor (+ive goes forwards)
		powerOutput = (1) * powerOutput;
		motor_move_voltage(_motorRight, powerOutput);
		break;
	case 1: // left wheel motor (+ive goes forwards)
		powerOutput = (1) * powerOutput;
		motor_move_voltage(_motorLeft, powerOutput);
		break;
	case 2: // robot arm motor (+ive raises arm)
		powerOutput = (1) * powerOutput;
		if (adi_digital_read(_armLimitLow) && powerOutput < 0)
		{
			motor_move_voltage(_motorArm, 0);
		}
		else if (adi_digital_read(_armLimitHigh) && powerOutput > 0)
		{
			motor_move_voltage(_motorArm, 0);
		}
		else
		{
			motor_move_voltage(_motorArm, powerOutput);
		}
		break;
	}
}

// __[ RESET SENSOR ]___________________________________________________
/**
 * @brief Resets the counts of a selected encoder to zero.
 * @param encoderName Options include: RightEncoder, LeftEncoder, or ArmEncoder
 */
void resetEncoder(int encoderName)
{

	switch (encoderName)
	{
	case 5: // right wheel encoder
		motor_tare_position(_encoderRight);
		break;
	case 6: // left wheel encoder
		motor_tare_position(_encoderLeft);
		break;
	case 9: // robot arm encoder
		motor_tare_position(_encoderArm);
		break;
	}
}

// __[ RESET TIMER ]____________________________________________________
/**
 * @brief Resets the elapsed time of a selected timer to zero.
 * @param timerSelect Options include: T_1, T_2, T_3, or T_4
 */
void resetTimer(int timerSelect)
{

	switch (timerSelect)
	{
	case 0: // timer #1
		T1_timer = millis();
		break;
	case 1: // timer #2
		T2_timer = millis();
		break;
	case 2: // timer #3
		T3_timer = millis();
		break;
	case 3: // timer #4
		T4_timer = millis();
		break;
	}
}

/// @brief Call after the student main code to stop all motors and prevent future motor usage.
void endOfProgram()
{
	motor_move(_motorLeft, 0);
	motor_move(_motorRight, 0);
	motor_move(_motorArm, 0);

	if (_stopflag == 0)
	{ // if program wasn't stopped by pressing the button
		_stopflag = 1;
		lcd_print(LCDLine8, "    Program ended normally.");
	}
}


// ----------------------------------- General functions ----------------------------------------

/// @brief Returns the lowest of two input numbers
/// @param num1 First input number to compare
/// @param num2 Second input number to compare
/// @return The lower of the two inputs
double min(double num1, double num2)
{
    if (num1 < num2)
    {
        return num1;
    }
    else
    {
        return num2;
    }
}


/// @brief Returns the highest of two input numbers
/// @param num1 First input number to compare
/// @param num2 Second input number to compare
/// @return The higher of the two inputs
double max(double num1, double num2)
{
    if (num1 > num2)
    {
        return num1;
    }
    else
    {
        return num2;
    }
}

/// @brief Implementation of the signum function. Returns 1 if the input number is +ve, -1 if negative, and 0 if it's 0.
/// @param input Input number
/// @return Sign of the input number (1 if positive, -1 if negative, and 0 if input is 0)
int sgn(double input)
{
    if (input == 0)
    {
        return 0;
    }
    else if (input > 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

// __[ SATURATE ]_______________________________________________________
/**
 * @brief Constrains an input value between the lower and upper input limits. If the input number is below the lower limit, the function will return the lower limit.
 * Similarly, if the input is higher than the upper limit, the function will return the upper limit. If the input number is between the two limits, the function will
 * return the input number unmodified.
 * @param input (double) Input number
 * @param lower (double) Lower limit of desired range
 * @param upper (double) Upper limit of desired range
 * @return (double) The saturated number guaranteed to be between the lower and upper inputs.
 */
double saturate(double input, double lower, double upper)
{

	if (input > upper)
	{
		return upper; // limit by upper bound
	}
	else if (input < lower)
	{
		return lower; // limit by lower bound
	}
	else
	{
		return input; // else leave unadjusted
	}
}