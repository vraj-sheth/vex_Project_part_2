/**
 * \file pros/motors.h
 *
 * Contains prototypes for the V5 Motor-related functions.
 *
 * Visit https://pros.cs.purdue.edu/v5/tutorials/topical/motors.html to learn
 * more.
 *
 * This file should not be modified by users, since it gets replaced whenever
 * a kernel upgrade occurs.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MOTORS_H_
#define _PROS_MOTORS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum motor_gearset_e {
	E_MOTOR_GEARSET_18 = 1,  // 18:1, 200 RPM, Green gear set
	E_MOTOR_GEARSET_INVALID = INT32_MAX
} motor_gearset_e_t;
/******************************************************************************/
/**                         Motor movement functions                         **/
/**                                                                          **/
/**          These functions allow programmers to make motors move           **/
/******************************************************************************/

/**
 * Sets the voltage for the motor from -127 to 127.
 *
 * This is designed to map easily to the input from the controller's analog
 * stick for simple opcontrol use. The actual behavior of the motor is analogous
 * to use of motor_move_voltage(), or motorSet() from the PROS 2 API.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 * \param voltage
 *        The new motor voltage from -127 to 127
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t motor_move(uint8_t port, int32_t voltage);

/**
 * Gets the velocity commanded to the motor by the user.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 *
 * \return The commanded motor velocity from +-100, +-200, or +-600, or PROS_ERR
 * if the operation failed, setting errno.
 */
int32_t motor_get_target_velocity(uint8_t port);

/**
 * Gets the power drawn by the motor in Watts.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 *
 * \return The motor's power draw in Watts or PROS_ERR_F if the operation
 * failed, setting errno.
 */
double motor_get_power(uint8_t port);



/******************************************************************************/
/**                        Motor telemetry functions                         **/
/**                                                                          **/
/**    These functions allow programmers to collect telemetry from motors    **/
/******************************************************************************/


/**
 * Gets the direction of movement for the motor.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 *
 * \return 1 for moving in the positive direction, -1 for moving in the
 * negative direction, or PROS_ERR if the operation failed, setting errno.
 */
int32_t motor_get_direction(uint8_t port);

/**
 * Gets the absolute position of the motor in its encoder units.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 *
 * \return The motor's absolute position in its encoder units or PROS_ERR_F
 * if the operation failed, setting errno.
 */
double motor_get_position(uint8_t port);

/******************************************************************************/
/**                      Motor configuration functions                       **/
/**                                                                          **/
/**  These functions allow programmers to configure the behavior of motors   **/
/******************************************************************************/

/**
 * Indicates the units used by the motor encoders.
 */
typedef enum motor_encoder_units_e {
	//E_MOTOR_ENCODER_DEGREES = 0,    // Position is recorded as angle in degrees
	                                // as a floating point number
	//E_MOTOR_ENCODER_ROTATIONS = 1,  // Position is recorded as angle in rotations
	                                // as a floating point number
	E_MOTOR_ENCODER_COUNTS = 2,     // Position is recorded as raw encoder ticks
	                                // as a whole number
	E_MOTOR_ENCODER_INVALID = INT32_MAX
} motor_encoder_units_e_t;

/**
 * Sets the "absolute" zero position of the motor to its current position.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t motor_tare_position(uint8_t port);

/**
 * Sets the reverse flag for the motor.
 *
 * This will invert its movements and the values returned for its position.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 * \param reverse
 *        True reverses the motor, false is default
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t motor_set_reversed(uint8_t port, const bool reverse);

/**
 * Sets one of motor_encoder_units_e_t for the motor encoder.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 * \param units
 *        The new motor encoder units
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t motor_set_encoder_units(uint8_t port, const motor_encoder_units_e_t units);

/**
 * Sets one of motor_gearset_e_t for the motor.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 * \param gearset
 *        The new motor gearset
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t motor_set_gearing(uint8_t port, const motor_gearset_e_t gearset);

/**
 * Gets the operation direction of the motor as set by the user.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (1-21).
 * ENODEV - The port cannot be configured as a motor
 *
 * \param port
 *        The V5 port number from 1-21
 *
 * \return 1 if the motor has been reversed and 0 if the motor was not reversed,
 * or PROS_ERR if the operation failed, setting errno.
 */
int32_t motor_is_reversed(uint8_t port);

#endif  // _PROS_MOTORS_H_
