/**
 * \file pros/misc.h
 *
 * Contains prototypes for miscellaneous functions pertaining to the controller,
 * battery, and competition control.
 *
 * Visit https://pros.cs.purdue.edu/v5/tutorials/topical/controller.html to
 * learn more.
 *
 * This file should not be modified by users, since it gets replaced whenever
 * a kernel upgrade occurs.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reservered.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MISC_H_
#define _PROS_MISC_H_

#include <stdint.h>

#define NUM_V5_PORTS (22)

/*
Given an id and a port, this macro sets the port 
variable based on the id and allows the mutex to take that port.

Returns error (in the function/scope it's in) if the controller
failed to connect or an invalid id is given.
*/

/**
 * Gets the current voltage of the battery, as reported by VEXos.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * EACCES - Another resource is currently trying to access the battery port.
 *
 * \return The current voltage of the battery
 */
int32_t battery_get_voltage(void);

/**
 * Gets the current capacity of the battery, as reported by VEXos.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * EACCES - Another resource is currently trying to access the battery port.
 *
 * \return The current capacity of the battery
 */
double battery_get_capacity(void);

#endif  // _PROS_MISC_H_
