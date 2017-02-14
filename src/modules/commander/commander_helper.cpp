/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file commander_helper.cpp
 * Commander helper functions implementations
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 */

#include <px4_defines.h>
#include <px4_posix.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/led_control.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include "commander_helper.h"
#include "DevMgr.hpp"

using namespace DriverFramework;

#define VEHICLE_TYPE_QUADROTOR 2
#define VEHICLE_TYPE_COAXIAL 3
#define VEHICLE_TYPE_HELICOPTER 4
#define VEHICLE_TYPE_HEXAROTOR 13
#define VEHICLE_TYPE_OCTOROTOR 14
#define VEHICLE_TYPE_TRICOPTER 15
#define VEHICLE_TYPE_VTOL_DUOROTOR 19
#define VEHICLE_TYPE_VTOL_QUADROTOR 20
#define VEHICLE_TYPE_VTOL_TILTROTOR 21
#define VEHICLE_TYPE_VTOL_RESERVED2 22
#define VEHICLE_TYPE_VTOL_RESERVED3 23
#define VEHICLE_TYPE_VTOL_RESERVED4 24
#define VEHICLE_TYPE_VTOL_RESERVED5 25

#define BLINK_MSG_TIME	7	// 3 fast blinks in ticks

bool is_multirotor(const struct vehicle_status_s *current_status)
{
	return ((current_status->system_type == VEHICLE_TYPE_QUADROTOR) ||
		(current_status->system_type == VEHICLE_TYPE_HEXAROTOR) ||
		(current_status->system_type == VEHICLE_TYPE_OCTOROTOR) ||
		(current_status->system_type == VEHICLE_TYPE_TRICOPTER));
}

bool is_rotary_wing(const struct vehicle_status_s *current_status)
{
	return is_multirotor(current_status) || (current_status->system_type == VEHICLE_TYPE_HELICOPTER)
		   || (current_status->system_type == VEHICLE_TYPE_COAXIAL);
}

bool is_vtol(const struct vehicle_status_s * current_status) {
	return (current_status->system_type == VEHICLE_TYPE_VTOL_DUOROTOR ||
		current_status->system_type == VEHICLE_TYPE_VTOL_QUADROTOR ||
		current_status->system_type == VEHICLE_TYPE_VTOL_TILTROTOR ||
		current_status->system_type == VEHICLE_TYPE_VTOL_RESERVED2 ||
		current_status->system_type == VEHICLE_TYPE_VTOL_RESERVED3 ||
		current_status->system_type == VEHICLE_TYPE_VTOL_RESERVED4 ||
		current_status->system_type == VEHICLE_TYPE_VTOL_RESERVED5);
}

static hrt_abstime blink_msg_end = 0;	// end time for currently blinking LED message, 0 if no blink message
static hrt_abstime tune_end = 0;		// end time of currently played tune, 0 for repeating tunes or silence
static int tune_current = TONE_STOP_TUNE;		// currently playing tune, can be interrupted after tune_end
static unsigned int tune_durations[TONE_NUMBER_OF_TUNES];

static DevHandle h_leds;
static DevHandle h_buzzer;
static led_control_s led_control = {};
static orb_advert_t led_control_pub = nullptr;

int buzzer_init()
{
	tune_end = 0;
	tune_current = 0;
	memset(tune_durations, 0, sizeof(tune_durations));
	tune_durations[TONE_NOTIFY_POSITIVE_TUNE] = 800000;
	tune_durations[TONE_NOTIFY_NEGATIVE_TUNE] = 900000;
	tune_durations[TONE_NOTIFY_NEUTRAL_TUNE] = 500000;
	tune_durations[TONE_ARMING_WARNING_TUNE] = 3000000;

	DevMgr::getHandle(TONEALARM0_DEVICE_PATH, h_buzzer);

	if (!h_buzzer.isValid()) {
		PX4_WARN("Buzzer: px4_open fail\n");
		return PX4_ERROR;
	}

	return PX4_OK;
}

void buzzer_deinit()
{
	DevMgr::releaseHandle(h_buzzer);
}

void set_tune_override(int tune)
{
	h_buzzer.ioctl(TONE_SET_ALARM, tune);
}

void set_tune(int tune)
{
	unsigned int new_tune_duration = tune_durations[tune];

	/* don't interrupt currently playing non-repeating tune by repeating */
	if (tune_end == 0 || new_tune_duration != 0 || hrt_absolute_time() > tune_end) {
		/* allow interrupting current non-repeating tune by the same tune */
		if (tune != tune_current || new_tune_duration != 0) {
			h_buzzer.ioctl(TONE_SET_ALARM, tune);
		}

		tune_current = tune;

		if (new_tune_duration != 0) {
			tune_end = hrt_absolute_time() + new_tune_duration;

		} else {
			tune_end = 0;
		}
	}
}

static void set_tune_and_rgbled(bool use_buzzer, rgbled_color_t color, rgbled_mode_t mode, int tune)
{
	rgbled_mode_and_color_t mode_color;
	mode_color.enabled = 0xFF;
	mode_color.mode = mode;
	mode_color.color = color;
	mode_color.prio = 0;
	mode_color.duration = BLINK_MSG_TIME;
	rgbled_set_mode_and_color(&mode_color);

	if (use_buzzer) {
		set_tune(tune);
	}
}

void tune_home_set(bool use_buzzer)
{
	set_tune_and_rgbled(use_buzzer, RGBLED_COLOR_GREEN, RGBLED_MODE_BLINK_FAST, TONE_HOME_SET);
}

void tune_mission_ok(bool use_buzzer)
{
	set_tune_and_rgbled(use_buzzer, RGBLED_COLOR_GREEN, RGBLED_MODE_BLINK_FAST, TONE_NOTIFY_NEUTRAL_TUNE);
}

void tune_mission_fail(bool use_buzzer)
{
	set_tune_and_rgbled(use_buzzer, RGBLED_COLOR_GREEN, RGBLED_MODE_BLINK_FAST, TONE_NOTIFY_NEGATIVE_TUNE);
}

/**
 * Blink green LED and play positive tune (if use_buzzer == true).
 */
void tune_positive(bool use_buzzer)
{
	set_tune_and_rgbled(use_buzzer, RGBLED_COLOR_GREEN, RGBLED_MODE_BLINK_FAST, TONE_NOTIFY_POSITIVE_TUNE);
}

/**
 * Blink white LED and play neutral tune (if use_buzzer == true).
 */
void tune_neutral(bool use_buzzer)
{
	set_tune_and_rgbled(use_buzzer, RGBLED_COLOR_WHITE, RGBLED_MODE_BLINK_FAST, TONE_NOTIFY_NEUTRAL_TUNE);
}

/**
 * Blink red LED and play negative tune (if use_buzzer == true).
 */
void tune_negative(bool use_buzzer)
{
	set_tune_and_rgbled(use_buzzer, RGBLED_COLOR_RED, RGBLED_MODE_BLINK_FAST, TONE_NOTIFY_NEGATIVE_TUNE);
}

void tune_failsafe(bool use_buzzer)
{
	set_tune_and_rgbled(use_buzzer, RGBLED_COLOR_PURPLE, RGBLED_MODE_BLINK_FAST, TONE_BATTERY_WARNING_FAST_TUNE);
}

int blink_msg_state()
{
	if (blink_msg_end == 0) {
		return 0;

	} else if (hrt_absolute_time() > blink_msg_end) {
		blink_msg_end = 0;
		return 2;

	} else {
		return 1;
	}
}

int led_init()
{
	blink_msg_end = 0;

	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_OFF;
	led_control.priority = 0;
	led_control.timestamp = hrt_absolute_time();
	led_control_pub = orb_advertise_queue(ORB_ID(led_control), &led_control, LED_UORB_QUEUE_LENGTH);

#ifndef CONFIG_ARCH_BOARD_RPI
	/* first open normal LEDs */
	DevMgr::getHandle(LED0_DEVICE_PATH, h_leds);

	if (!h_leds.isValid()) {
		PX4_WARN("LED: getHandle fail\n");
		return PX4_ERROR;
	}

	/* the blue LED is only available on FMUv1 & AeroCore but not FMUv2 */
	(void)h_leds.ioctl(LED_ON, LED_BLUE);

	/* switch blue off */
	led_off(LED_BLUE);

	/* we consider the amber led mandatory */
	if (h_leds.ioctl(LED_ON, LED_AMBER)) {
		PX4_WARN("Amber LED: ioctl fail\n");
		return PX4_ERROR;
	}

	/* switch amber off */
	led_off(LED_AMBER);
#endif

	return 0;
}

void led_deinit()
{
	orb_unadvertise(led_control_pub);
#ifndef CONFIG_ARCH_BOARD_RPI
	DevMgr::releaseHandle(h_leds);
#endif
}

int led_toggle(int led)
{
	return h_leds.ioctl(LED_TOGGLE, led);
}

int led_on(int led)
{
	return h_leds.ioctl(LED_ON, led);
}

int led_off(int led)
{
	return h_leds.ioctl(LED_OFF, led);
}

void rgbled_set_color(rgbled_color_t color)
{
	led_control.mode = led_control_s::MODE_ON;
	switch(color) {
		case RGBLED_COLOR_OFF:
			led_control.mode = led_control_s::MODE_OFF;
			break;
		case RGBLED_COLOR_RED:
			led_control.color = led_control_s::COLOR_RED;
			break;
		case RGBLED_COLOR_YELLOW:
			led_control.color = led_control_s::COLOR_YELLOW;
			break;
		case RGBLED_COLOR_PURPLE:
			led_control.color = led_control_s::COLOR_PURPLE;
			break;
		case RGBLED_COLOR_GREEN:
			led_control.color = led_control_s::COLOR_GREEN;
			break;
		case RGBLED_COLOR_BLUE:
			led_control.color = led_control_s::COLOR_BLUE;
			break;
		case RGBLED_COLOR_WHITE:
			led_control.color = led_control_s::COLOR_WHITE;
			break;
		case RGBLED_COLOR_AMBER:
			led_control.color = led_control_s::COLOR_AMBER;
			break;
		default:
			break;
	}
	led_control.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(led_control), led_control_pub, &led_control);
}

void rgbled_set_mode(rgbled_mode_t mode)
{
	switch(mode) {
		case RGBLED_MODE_OFF:
			led_control.mode = led_control_s::MODE_OFF;
			break;
		case RGBLED_MODE_ON:
		case RGBLED_MODE_BREATHE:
			led_control.mode = led_control_s::MODE_ON;
			break;
		case RGBLED_MODE_BLINK_SLOW:
			led_control.mode = led_control_s::MODE_BLINK_SLOW;
			break;
		case RGBLED_MODE_BLINK_NORMAL:
			led_control.mode = led_control_s::MODE_BLINK_NORMAL;
			break;
		case RGBLED_MODE_BLINK_FAST:
			led_control.mode = led_control_s::MODE_BLINK_FAST;
			break;
		default:
			break;
	}
	led_control.num_blinks = 3;
	led_control.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(led_control), led_control_pub, &led_control);
}

void rgbled_set_mode_and_color(rgbled_mode_and_color_t *mode_color)
{
	switch(mode_color->mode) {
		case RGBLED_MODE_OFF:
			led_control.mode = led_control_s::MODE_OFF;
			break;
		case RGBLED_MODE_ON:
		case RGBLED_MODE_BREATHE:
			led_control.mode = led_control_s::MODE_ON;
			break;
		case RGBLED_MODE_BLINK_SLOW:
			led_control.mode = led_control_s::MODE_BLINK_SLOW;
			break;
		case RGBLED_MODE_BLINK_NORMAL:
			led_control.mode = led_control_s::MODE_BLINK_NORMAL;
			break;
		case RGBLED_MODE_BLINK_FAST:
			led_control.mode = led_control_s::MODE_BLINK_FAST;
			break;
		default:
			break;
	}
	led_control.num_blinks = 3;
	switch(mode_color->color) {
		case RGBLED_COLOR_OFF:
			led_control.mode = led_control_s::MODE_OFF;
			break;
		case RGBLED_COLOR_RED:
			led_control.color = led_control_s::COLOR_RED;
			break;
		case RGBLED_COLOR_YELLOW:
			led_control.color = led_control_s::COLOR_YELLOW;
			break;
		case RGBLED_COLOR_PURPLE:
			led_control.color = led_control_s::COLOR_PURPLE;
			break;
		case RGBLED_COLOR_GREEN:
			led_control.color = led_control_s::COLOR_GREEN;
			break;
		case RGBLED_COLOR_BLUE:
			led_control.color = led_control_s::COLOR_BLUE;
			break;
		case RGBLED_COLOR_WHITE:
			led_control.color = led_control_s::COLOR_WHITE;
			break;
		case RGBLED_COLOR_AMBER:
			led_control.color = led_control_s::COLOR_AMBER;
			break;
		default:
			break;
	}
	led_control.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(led_control), led_control_pub, &led_control);
}

