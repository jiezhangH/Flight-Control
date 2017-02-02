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
 * @file status_display.h
 * Status Display decouple the LED and tune form the original commander
 *
 * @author Simone Guscetti <simone@px4.io>
 */

#pragma once

#include "subscriber_handler.h"

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/main_led.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

#define CPU_OVERLOAD_VALUE 0.80f
#define RAM_OVERLOAD_VALUE 0.98f
/**< wait for hotplug sensors to come online for upto 8 seconds */
#define HOTPLUG_SENS_TIMEOUT (8 * 1000 * 1000)


namespace status
{

class StatusDisplay
{
public:
	void process(events::SubscriberHandler &sh);

protected:
	bool check_for_updates(events::SubscriberHandler &sh);
	void set_main_led();
	void publish();
	// TODO: review if there is a better variant that allocate this in the
	// memory
	struct battery_status_s _battery_status = {};
	struct cpuload_s _cpu_load = {};
	struct vehicle_status_s _vehicle_status = {};
	struct vehicle_status_flags_s _vehicle_status_flags = {};

	struct main_led_s _rgb;

private:
	orb_advert_t _main_led_pub = nullptr;
	hrt_abstime _status_display_uptime = 0;
};

} /* status */
