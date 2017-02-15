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
	_led_control.num_blinks = 3;
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
	bool is_armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	bool reset_leds = false;

	if (is_armed != _arming_state) {
		if (!is_armed) {
			reset_leds = true;
		}

		_arming_state = is_armed;
	}

	if (is_armed) {
		// check attitude
		matrix::Eulerf euler = matrix::Quatf(_vehicle_attitude.q);

		uint8_t led_mask_blinking = 0;
		uint8_t led_mask_normal = 0;
		bool is_tilted = false;

		// fly left
		if (fabsf(euler.theta() * 57.3f) < 6.0f && (euler.phi() * 57.3f) < -5.0f) {
			led_mask_normal = (1 << 0) | (1 << 1) | (1 << 2);
			led_mask_blinking = (1 << 3) | (1 << 4) | (1 << 5);
			is_tilted = true;
		}

		// fly right
		if (fabsf(euler.theta() * 57.3f) < 6.0f && (euler.phi() * 57.3f) > 5.0f) {
			led_mask_normal = (1 << 3) | (1 << 4) | (1 << 5);
			led_mask_blinking = (1 << 0) | (1 << 1) | (1 << 2);
			is_tilted = true;
		}

		// fly forward
		if (fabsf(euler.phi() * 57.3f) < 6.0f && (euler.theta() * 57.3f) < -5.0f) {
			led_mask_normal = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);
			led_mask_blinking = (1 << 2) | (1 << 3);
			is_tilted = true;
		}

		// fly back
		if (fabsf(euler.phi() * 57.3f) < 6.0f && (euler.theta() * 57.3f) > 5.0f) {
			led_mask_normal = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);
			led_mask_blinking = (1 << 0) | (1 << 5);
			is_tilted = true;
		}

		if (is_tilted) {
			_led_control.led_mask = led_mask_normal;
			_led_control.mode = led_control_s::MODE_ON;
			_led_control.color = led_control_s::COLOR_GREEN;
			publish();
			_led_control.led_mask = led_mask_blinking;
			_led_control.mode = led_control_s::MODE_BLINK_NORMAL;
			_led_control.color = led_control_s::COLOR_PURPLE;
			_led_control.num_blinks = 0;
			publish();

		} else {
			reset_leds = true;
		}
	}

	if (reset_leds) {
		_led_control.led_mask = 0xff;
		_led_control.mode = led_control_s::MODE_DISABLED;
		publish();
	}

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
