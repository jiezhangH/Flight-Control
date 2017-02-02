#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

// Definition of the bitfield
#define VEHICLE_COMMAND_MASK (0x01 << 0)
#define VEHICLE_STATUS_MASK (0x01 << 1)
#define VEHICLE_STATUS_FLAGS_MASK (0x01 << 2)
#define BATTERY_STATUS_MASK (0x01 << 3)
#define CPU_LOAD_MASK (0x01 << 4)

namespace events
{

class SubscriberHandler
{
public:
	void subscribe();
	void unsubscribe();
	void check_for_updates();
	// TODO change to a generalized get subscriber method
	int get_battery_status_sub();
	int get_cpuload_sub();
	int get_vehicle_command_sub();
	int get_vehicle_status_sub();
	int get_vehicle_status_flags_sub();
	// TODO: incorporate an add_topic method, this will push back the sub handler
	// in the subscriber vector

	uint32_t get_update_bitfield();

private:
	// TODO: incorporate the subscriber into a vector of int
	int _battery_status_sub = -1;
	int _cpuload_sub = -1;
	int _vehicle_command_sub = -1;
	int _vehicle_status_sub = -1;
	int _vehicle_status_flags_sub = -1;

	uint32_t _update_bitfield = 0;
};

} /* events */
