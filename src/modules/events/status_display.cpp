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
 * Status Display decouple the LED and tune form the original commander
 *
 * @author Simone Guscetti <simone@px4.io>
 */

#include "status_display.h"
#include <px4_log.h>

using namespace status;

StatusDisplay::StatusDisplay(const events::SubscriberHandler &subscriber_handler)
	: _subscriber_handler(subscriber_handler)
{
}

bool StatusDisplay::check_for_updates()
{
	if (_subscriber_handler.battery_status_updated()) {
		orb_copy(ORB_ID(battery_status), _subscriber_handler.get_battery_status_sub(), &_battery_status);
	}

	if (_subscriber_handler.cpuload_updated()) {
		orb_copy(ORB_ID(cpuload), _subscriber_handler.get_cpuload_sub(), &_cpu_load);
	}

	if (_subscriber_handler.vehicle_status_flags_updated()) {
		orb_copy(ORB_ID(vehicle_status_flags), _subscriber_handler.get_vehicle_status_flags_sub(), &_vehicle_status_flags);
	}

	// right now the criteria is to have some vehicle_status updates to process
	// the LED status
	if (_subscriber_handler.vehicle_status_updated()) {
		orb_copy(ORB_ID(vehicle_status), _subscriber_handler.get_vehicle_status_sub(), &_vehicle_status);
		return true;
	}

	return false;
}

void StatusDisplay::process()
{
	if (_status_display_uptime == 0) {
		_status_display_uptime = hrt_absolute_time();
	}

	// if any update to the vehicle status topic exit
	if (!check_for_updates()) {
		return;
	}

	set_main_led();
	publish();
}

void StatusDisplay::set_main_led()
{
	static hrt_abstime overload_start = 0;
	bool hotplug_timeout = hrt_elapsed_time(&_status_display_uptime) > HOTPLUG_SENS_TIMEOUT;
	bool overload = (_cpu_load.load > CPU_OVERLOAD_VALUE) || (_cpu_load.ram_usage > RAM_OVERLOAD_VALUE);

	if (overload_start == 0 && overload) {
		overload_start = hrt_absolute_time();

	} else if (!overload) {
		overload_start = 0;
	}

	// Ported from commander
	bool set_normal_color = false;

	int overload_warn_delay = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1000 : 250000;

	/* set mode */
	if (overload && ((hrt_absolute_time() - overload_start) > overload_warn_delay)) {
		_rgb.mode = main_led_s::LED_MODE_BLINK_FAST;
		_rgb.color = main_led_s::LED_COLOR_PURPLE;
		set_normal_color = false;

	} else if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		_rgb.mode = main_led_s::LED_MODE_ON;
		set_normal_color = true;

	} else if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR ||
		   (!((_vehicle_status_flags.conditions & vehicle_status_flags_s::CONDITION_SYSTEM_SENSORS_INITIALIZED_MASK)
		      == vehicle_status_flags_s::CONDITION_SYSTEM_SENSORS_INITIALIZED_MASK) && hotplug_timeout)) {
		_rgb.mode = main_led_s::LED_MODE_BLINK_FAST;
		_rgb.color = main_led_s::LED_COLOR_RED;
		set_normal_color = false;

	} else if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
		_rgb.mode = main_led_s::LED_MODE_BREATHE;
		set_normal_color = true;

	} else if (!((_vehicle_status_flags.conditions & vehicle_status_flags_s::CONDITION_SYSTEM_SENSORS_INITIALIZED_MASK)
		     == vehicle_status_flags_s::CONDITION_SYSTEM_SENSORS_INITIALIZED_MASK) && !hotplug_timeout) {
		_rgb.mode = main_led_s::LED_MODE_BREATHE;
		set_normal_color = true;

	} else {	// STANDBY_ERROR and other states
		_rgb.mode = main_led_s::LED_MODE_BLINK_NORMAL;
		_rgb.color = main_led_s::LED_COLOR_RED;
		set_normal_color = false;
	}

	if (set_normal_color) {
		/* set color */
		if (_vehicle_status.failsafe) {
			_rgb.color = main_led_s::LED_COLOR_PURPLE;

		} else if (_battery_status.warning == battery_status_s::BATTERY_WARNING_LOW) {
			_rgb.color = main_led_s::LED_COLOR_YELLOW;

		} else if (_battery_status.warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
			_rgb.color = main_led_s::LED_COLOR_RED;

		} else {
			// TODO: check this condition
			if (((_vehicle_status_flags.conditions & vehicle_status_flags_s::CONDITION_HOME_POSITION_VALID_MASK) ==
			     vehicle_status_flags_s::CONDITION_HOME_POSITION_VALID_MASK) &&
			    ((_vehicle_status_flags.conditions & vehicle_status_flags_s::CONDITION_GLOBAL_POSITION_VALID_MASK) ==
			     vehicle_status_flags_s::CONDITION_GLOBAL_POSITION_VALID_MASK)) {
				_rgb.color = main_led_s::LED_COLOR_GREEN;

			} else {
				_rgb.color = main_led_s::LED_COLOR_BLUE;
			}
		}
	}
}

void StatusDisplay::publish()
{
	if (_main_led_pub != nullptr) {
		orb_publish(ORB_ID(main_led), _main_led_pub, &_rgb);

	} else {
		_main_led_pub =  orb_advertise(ORB_ID(main_led), &_rgb);
	}
}
