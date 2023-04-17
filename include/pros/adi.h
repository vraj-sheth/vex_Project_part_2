/**
 * \file pros/adi.h
 *
 * Contains prototypes for interfacing with the ADI.
 *
 * Visit https://pros.cs.purdue.edu/v5/tutorials/topical/adi.html to learn more.
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

#ifndef _PROS_ADI_H_
#define _PROS_ADI_H_

#include <stdbool.h>
#include <stdint.h>
#ifndef PROS_ERR
#define PROS_ERR (INT32_MAX)
#endif



/**
 * Represents the port type for an ADI port.
 */
typedef enum adi_port_config_e {
	E_ADI_ANALOG_IN = 0,
	E_ADI_ANALOG_OUT = 1,
	E_ADI_DIGITAL_IN = 2,
	E_ADI_DIGITAL_OUT = 3,

#define _DEPRECATE_DIGITAL_IN __attribute__((deprecated("use E_ADI_DIGITAL_IN instead"))) = E_ADI_DIGITAL_IN
#define _DEPRECATE_ANALOG_IN __attribute__((deprecated("use E_ADI_ANALOG_IN instead"))) = E_ADI_ANALOG_IN

	
	E_ADI_LEGACY_LINE_SENSOR _DEPRECATE_ANALOG_IN,
	E_ADI_LEGACY_LIGHT_SENSOR _DEPRECATE_ANALOG_IN,

#undef _DEPRECATE_DIGITAL_IN
#undef _DEPRECATE_ANALOG_IN

	E_ADI_LEGACY_ULTRASONIC = 15,

	E_ADI_TYPE_UNDEFINED = 255,
	E_ADI_ERR = PROS_ERR
} adi_port_config_e_t;

#define INTERNAL_ADI_PORT 22
#define NUM_ADI_PORTS 8

/******************************************************************************/
/**                         General ADI Use Functions                        **/
/**                                                                          **/
/**       These functions allow for interaction with any ADI port type       **/
/******************************************************************************/

/**
 * Gets the configuration for the given ADI port.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports.
 *
 * \param port
 *        The ADI port number (from 1-8, 'a'-'h', 'A'-'H') for which to return
 *        the configuration
 *
 * \return The ADI configuration for the given port
 */
adi_port_config_e_t adi_port_get_config(uint8_t port);

/**
 * Gets the value for the given ADI port.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports.
 *
 * \param port
 *        The ADI port number (from 1-8, 'a'-'h', 'A'-'H') for which the value
 *        will be returned
 *
 * \return The value stored for the given port
 */
int32_t adi_port_get_value(uint8_t port);

/**
 * Configures an ADI port to act as a given sensor type.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports.
 *
 * \param port
 *        The ADI port number (from 1-8, 'a'-'h', 'A'-'H') to configure
 * \param type
 *        The configuration type for the port
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t adi_port_set_config(uint8_t port, adi_port_config_e_t type);

/**
 * Sets the value for the given ADI port.
 *
 * This only works on ports configured as outputs, and the behavior will change
 * depending on the configuration of the port.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO  - The given value is not within the range of ADI Ports.
 *
 * \param port
 *        The ADI port number (from 1-8, 'a'-'h', 'A'-'H') for which the value
 *        will be set
 * \param value
 *        The value to set the ADI port to
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t adi_port_set_value(uint8_t port, int32_t value);

/******************************************************************************/
/**                      PROS 2 Compatibility Functions                      **/
/**                                                                          **/
/**     These functions provide similar functionality to the PROS 2 API      **/
/******************************************************************************/

/**
 * Used for adi_digital_write() to specify a logic HIGH state to output.
 *
 * In reality, using any non-zero expression or "true" will work to set a pin to
 * HIGH.
 */
#define HIGH 1
/**
 * Used for adi_digital_write() to specify a logic LOW state to output.
 *
 * In reality, using a zero expression or "false" will work to set a pin to LOW.
 */
#define LOW 0

/**
 * adi_pin_mode() state for a digital input.
 */
#define INPUT 0x00
/**
 * adi_pin_mode() state for a digital output.
 */
#define OUTPUT 0x01
/**
 * adi_pin_mode() state for an analog input.
 */
#define INPUT_ANALOG 0x02

/**
 * adi_pin_mode() state for an analog output.
 */
#define OUTPUT_ANALOG 0x03


/**
 * Gets the 12-bit value of the specified port.
 *
 * The value returned is undefined if the analog pin has been switched to a
 * different mode.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports
 * EADDRINUSE - The port is not configured as an analog input
 *
 * \param port
 *        The ADI port (from 1-8, 'a'-'h', 'A'-'H') for which the value will be
 *        returned
 *
 * \return The analog sensor value, where a value of 0 reflects an input voltage
 * of nearly 0 V and a value of 4095 reflects an input voltage of nearly 5 V
 */
int32_t adi_analog_read(uint8_t port);


/**
 * Gets the digital value (1 or 0) of a port configured as a digital input.
 *
 * If the port is configured as some other mode, the digital value which
 * reflects the current state of the port is returned, which may or may not
 * differ from the currently set value. The return value is undefined for ports
 * configured as any mode other than a Digital Input.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports
 * EADDRINUSE - The port is not configured as a digital input
 *
 * \param port
 *        The ADI port to read (from 1-8, 'a'-'h', 'A'-'H')
 *
 * \return True if the pin is HIGH, or false if it is LOW
 */
int32_t adi_digital_read(uint8_t port);

/**
 * Sets the digital value (1 or 0) of a port configured as a digital output.
 *
 * If the port is configured as some other mode, behavior is undefined.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports
 * EADDRINUSE - The port is not configured as a digital output
 *
 * \param port
 *        The ADI port to read (from 1-8, 'a'-'h', 'A'-'H')
 * \param value
 *        An expression evaluating to "true" or "false" to set the output to
 *        HIGH or LOW respectively, or the constants HIGH or LOW themselves
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t adi_digital_write(uint8_t port, bool value);

/**
 * Configures the port as an input or output with a variety of settings.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports
 *
 * \param port
 *        The ADI port to read (from 1-8, 'a'-'h', 'A'-'H')
 * \param mode
 *        One of INPUT, INPUT_ANALOG, INPUT_FLOATING, OUTPUT, or OUTPUT_OD
 *
 * \return 1 if the operation was successful or PROS_ERR if the operation
 * failed, setting errno.
 */
int32_t adi_pin_mode(uint8_t port, uint8_t mode);

/**
 * Reference type for an initialized ultrasonic.
 *
 * This merely contains the port number for the ultrasonic, unlike its use as an
 * object to store ultrasonic data in PROS 2.
 */
typedef int32_t adi_ultrasonic_t;

/**
 * Gets the current ultrasonic sensor value in centimeters.
 *
 * If no object was found, zero is returned. If the ultrasonic sensor was never
 * started, the return value is undefined. Round and fluffy objects can cause
 * inaccurate values to be returned.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports
 * EADDRINUSE - The port is not configured as an ultrasonic
 *
 * \param ult
 *        The adi_ultrasonic_t object from adi_ultrasonic_init() to read
 *
 * \return The distance to the nearest object in m^-4 (10000 indicates 1 meter),
 * measured from the sensor's mounting points.
 */
int32_t adi_ultrasonic_get(adi_ultrasonic_t ult);

/**
 * Creates an ultrasonic object and configures the specified ports accordingly.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of ADI Ports
 * EADDRINUSE - The port is not configured as an ultrasonic
 *
 * \param port_ping
 *        The port connected to the orange OUTPUT cable. This should be in port
 *        1, 3, 5, or 7 ('A', 'C', 'E', 'G').
 * \param port_echo
 *        The port connected to the yellow INPUT cable. This should be in the
 *        next highest port following port_ping.
 *
 * \return An adi_ultrasonic_t object to be stored and used for later calls to
 * ultrasonic functions
 */
adi_ultrasonic_t adi_ultrasonic_init(uint8_t port_ping, uint8_t port_echo);

#endif  // _PROS_ADI_H_