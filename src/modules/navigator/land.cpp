/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file land.cpp
 *
 * Helper class to land at the current position
 *
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>
#include <float.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>

#include "land.h"
#include "navigator.h"

#define DELAY_SIGMA	0.01f

Land::Land(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_land_state(LAND_STATE_NONE),
	_param_land_delay(this, "LAND_LOI_DELAY", false),
	_param_descend_alt(this, "MPC_LAND_ALT1", false)

{
	/* load initial params */
	updateParams();
	/* initial reset */
	on_inactive();
}

Land::~Land()
{
}

void
Land::on_inactive()
{
	_land_state = LAND_STATE_NONE;
}

void
Land::on_activation()
{
	set_current_position_item(&_mission_item);

	vehicle_command_s cmd{};
	// Set gimbal to default orientation
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE;
	cmd.timestamp = hrt_absolute_time();
	orb_advert_t pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
	(void)orb_unadvertise(pub);

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	mission_apply_limitation(&_mission_item);
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	/* for safety reasons don't go into LAND if landed */
	if (_navigator->get_land_detected()->landed) {
		_land_state = LAND_STATE_LANDED;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Already landed, not executing Landing");

		/* if not landed, brake first */

	} else {
		_land_state = LAND_STATE_BRAKE;
	}

	set_autoland_item();
}

void
Land::on_active()
{
	// Reset the position to the current position if the landing
	// sequence gets overriden by the user at any given point in time

	if (_navigator->get_control_mode()->flag_control_updated) {
		_mission_item.lat = _navigator->get_global_position()->lat;
		_mission_item.lon = _navigator->get_global_position()->lon;
		_mission_item.yaw = _navigator->get_global_position()->yaw;
	}

	if (_land_state != LAND_STATE_LANDED && is_mission_item_reached()) {
		advance_land();
		set_autoland_item();
	}
}

void
Land::set_autoland_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	switch (_land_state) {

	case LAND_STATE_BRAKE: {

			/* slow down before climbing */
			set_brake_item(&_mission_item);

			break;
		}

	case LAND_STATE_LOITER: {
			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;

			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();

			if ((int)_navigator->get_attitude_setpoint()->landing_gear == (int)vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN) {
				_mission_item.time_inside = 0.0f;

			} else {
				_mission_item.time_inside = _param_land_delay.get() < 0.0f ? 0.0f : _param_land_delay.get();
			}

			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = true;
			_mission_item.force_velocity = false;


			_navigator->set_can_loiter_at_sp(true);

			break;
		}

	case LAND_STATE_DESCEND: {
			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = _navigator->get_home_position()->alt + _param_descend_alt.get();
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			/* disable previous setpoint to prevent drift */
			pos_sp_triplet->previous.valid = false;
			break;
		}

	case LAND_STATE_LAND: {
			set_land_item(&_mission_item, true);
			_mission_item.yaw = NAN;
			break;
		}

	case LAND_STATE_LANDED: {
			set_idle_item(&_mission_item);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_apply_limitation(&_mission_item);
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
Land::advance_land()
{
	switch (_land_state) {
	case LAND_STATE_BRAKE:
		_land_state = LAND_STATE_LOITER;

		break;

	case LAND_STATE_LOITER: {
			if (_navigator->get_global_position()->alt > _navigator->get_home_position()->alt + _param_descend_alt.get()) {
				_land_state = LAND_STATE_DESCEND;

			} else {
				_land_state = LAND_STATE_LAND;
			}

			break;
		}

	case LAND_STATE_DESCEND:
		_land_state = LAND_STATE_LAND;

		break;

	case LAND_STATE_LAND:
		_land_state = LAND_STATE_LANDED;

		break;

	default:
		break;
	}
}
