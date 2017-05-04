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
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	/* for safety reasons don't go into LAND if landed */
	if (_navigator->get_land_detected()->landed) {
		_land_state = LAND_STATE_LANDED;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Already landed, not executing Landing");

		/* if not landed, loiter first */

	} else {
		_land_state = LAND_STATE_LOITER;
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

			/* we try to predict a waypoint such that the vehicle has enough time to slow down
			 * TODO:
			 * - navigator needs to be smarter: mission item should incorporate velocity demand (triplets already do)
			 * such that we can demand zero velocity instead of predicting forward
			 * - rtl and land share same logic -> combine */

			double lat_predict;
			double lon_predict;
			float time_to_travel = 1.6f;
			add_vector_to_global_position(_navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
						      _navigator->get_global_position()->vel_n *  time_to_travel,
						      _navigator->get_global_position()->vel_e * time_to_travel,
						      &lat_predict, &lon_predict);


			_mission_item.lat = lat_predict;
			_mission_item.lon = lon_predict;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = false;

			break;

		}

	case LAND_STATE_LOITER: {
			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;

			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = _param_land_delay.get() < 0.0f ? 0.001f : _param_land_delay.get();
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = true;

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
