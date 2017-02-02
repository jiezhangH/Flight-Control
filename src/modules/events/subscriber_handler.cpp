
#include "subscriber_handler.h"

#include <px4_log.h>

using namespace events;

void SubscriberHandler::subscribe()
{
	if (_battery_status_sub < 0) {
		_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	}

	if (_cpuload_sub < 0) {
		_cpuload_sub = orb_subscribe(ORB_ID(cpuload));
	}

	if (_vehicle_command_sub < 0) {
		_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	if (_vehicle_status_sub < 0) {
		_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	}
}

void SubscriberHandler::unsubscribe()
{
	if (_battery_status_sub >= 0) {
		orb_unsubscribe(_battery_status_sub);
		_battery_status_sub = -1;
	}

	if (_cpuload_sub >= 0) {
		orb_unsubscribe(_cpuload_sub);
		_cpuload_sub = -1;
	}

	if (_vehicle_command_sub >= 0) {
		orb_unsubscribe(_vehicle_command_sub);
		_vehicle_command_sub = -1;
	}

	if (_vehicle_status_sub >= 0) {
		orb_unsubscribe(_vehicle_status_sub);
		_vehicle_status_sub = -1;
	}
}

void SubscriberHandler::check_for_updates()
{
	bool updated;
	_update_bitfield = 0;
	orb_check(_vehicle_command_sub, &updated);

	if (updated) {
		_update_bitfield |= VEHICLE_COMMAND_MASK;
	}

	updated = false;
	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		_update_bitfield |= VEHICLE_STATUS_MASK;
	}

	updated = false;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		_update_bitfield |= BATTERY_STATUS_MASK;
	}

	updated = false;
	orb_check(_cpuload_sub, &updated);

	if (updated) {
		_update_bitfield |= CPU_LOAD_MASK;
	}
}

int SubscriberHandler::get_battery_status_sub()
{
	return _battery_status_sub;
}

int SubscriberHandler::get_cpuload_sub()
{
	return _cpuload_sub;
}

int SubscriberHandler::get_vehicle_command_sub()
{
	return _vehicle_command_sub;
}

int SubscriberHandler::get_vehicle_status_sub()
{
	return _vehicle_status_sub;
}

int SubscriberHandler::get_vehicle_status_flags_sub()
{
	return _vehicle_status_flags_sub;
}

uint32_t SubscriberHandler::get_update_bitfield()
{
	return _update_bitfield;
}
