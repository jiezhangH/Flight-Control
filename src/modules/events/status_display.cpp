/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file status_display.cpp
 * Status Display: this decouples the LED and tune logic from the control logic in commander
 *
 * @author Simone Guscetti <simone@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "status_display.h"
#include <px4_log.h>
#include <drivers/drv_led.h>
#include <matrix/math.hpp>

using namespace status;

StatusDisplay::StatusDisplay(const events::SubscriberHandler &subscriber_handler)
	: _subscriber_handler(subscriber_handler)
{
	// set the base color
	_led_control.priority = 0;
	_led_control.led_mask = 0xff;
	_led_control.color = led_control_s::COLOR_CYAN;
	_led_control.mode = led_control_s::MODE_ON;
	publish();

	_led_control.priority = 1;
	_led_control.num_blinks = 0;	// infinite blinking
}

bool StatusDisplay::check_for_updates()
{
	bool got_updates = false;

	if (_subscriber_handler.battery_status_updated()) {
		orb_copy(ORB_ID(battery_status), _subscriber_handler.get_battery_status_sub(), &_battery_status);
		got_updates = true;
	}

	if (_subscriber_handler.cpuload_updated()) {
		orb_copy(ORB_ID(cpuload), _subscriber_handler.get_cpuload_sub(), &_cpu_load);
		got_updates = true;
	}

	if (_subscriber_handler.vehicle_status_flags_updated()) {
		orb_copy(ORB_ID(vehicle_status_flags), _subscriber_handler.get_vehicle_status_flags_sub(), &_vehicle_status_flags);
		got_updates = true;
	}

	if (_subscriber_handler.vehicle_status_updated()) {
		orb_copy(ORB_ID(vehicle_status), _subscriber_handler.get_vehicle_status_sub(), &_vehicle_status);
		got_updates = true;
	}

	if (_subscriber_handler.vehicle_attitude_updated()) {
		orb_copy(ORB_ID(vehicle_attitude), _subscriber_handler.get_vehicle_attitude_sub(), &_vehicle_attitude);
		got_updates = true;
	}

	return got_updates;
}

void StatusDisplay::process()
{
	if (!check_for_updates()) {
		return;
	}

	set_leds();
}

void StatusDisplay::set_leds()
{
	bool gps_lock_valid = (_vehicle_status_flags.conditions & vehicle_status_flags_s::CONDITION_GLOBAL_POSITION_VALID_MASK)
			      == vehicle_status_flags_s::CONDITION_GLOBAL_POSITION_VALID_MASK;
	bool home_position_valid = (_vehicle_status_flags.conditions &
				    vehicle_status_flags_s::CONDITION_HOME_POSITION_VALID_MASK) ==
				   vehicle_status_flags_s::CONDITION_HOME_POSITION_VALID_MASK;
	int nav_state = _vehicle_status.nav_state;

	// try to publish the static LED for the first 10s
	// this avoid the problem if a LED driver did not subscribe to the topic yet
	if (hrt_absolute_time() < 10 * 1000000) {
		// set the base color for motor 1
		_led_control.led_mask = (1 << 1);
		_led_control.color = led_control_s::COLOR_GREEN;
		_led_control.mode = led_control_s::MODE_ON;
		publish();

		// set the base color for motor 2 and 3
		_led_control.led_mask = (1 << 2) | (1 << 3);
		_led_control.color = led_control_s::COLOR_WHITE;
		_led_control.mode = led_control_s::MODE_ON;
		publish();

		// set the base color for motor 4
		_led_control.led_mask = (1 << 4);
		_led_control.color = led_control_s::COLOR_RED;
		_led_control.mode = led_control_s::MODE_ON;
		publish();
	}

	// set the led mask for the status led which are number 0 and 5
	_led_control.led_mask = (1 << 0) | (1 << 5);

	// choose color depending on the nav state
	if (nav_state == vehicle_status_s::NAVIGATION_STATE_SMART) {
		_led_control.color = led_control_s::COLOR_GREEN;

	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		_led_control.color = led_control_s::COLOR_PURPLE;

	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL) {
		_led_control.color = led_control_s::COLOR_CYAN;

	} else {
		_led_control.color = led_control_s::COLOR_PURPLE;
	}

	// blink if no GPS and home are set
	if (gps_lock_valid && home_position_valid) {
		_led_control.mode = led_control_s::MODE_ON;

	} else {
		_led_control.mode = led_control_s::MODE_BLINK_NORMAL;
	}

	if (nav_state != _old_nav_state || gps_lock_valid != _old_gps_lock_valid
	    || home_position_valid != home_position_valid) {
		publish();
	}

	// copy actual state
	_old_nav_state = nav_state;
	_old_gps_lock_valid = gps_lock_valid;
	_old_home_position_valid = home_position_valid;
}

void StatusDisplay::publish()
{
	_led_control.timestamp = hrt_absolute_time();

	if (_led_control_pub != nullptr) {
		orb_publish(ORB_ID(led_control), _led_control_pub, &_led_control);

	} else {
		_led_control_pub =  orb_advertise_queue(ORB_ID(led_control), &_led_control, LED_UORB_QUEUE_LENGTH);
	}
}
