/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file hc_sr04.cpp
 *
 * Driver for the hc_sr04 sonar range finders .
 */

#include <nuttx/config.h>

#include <drivers/device/device.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <platforms/px4_getopt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/px4_macros.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <controllib/blocks.hpp>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>

#include <lib/conversion/rotation.h>

#include <board_config.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_pwm_output.h>

#include <DevMgr.hpp>

/* Configuration Constants */
#define SR04_DEVICE_PATH	"/dev/hc_sr04"

#define SUBSYSTEM_TYPE_RANGEFINDER 131072
/* Device limits */
#define SR04_MIN_DISTANCE 	(0.40f)
#define SR04_MAX_DISTANCE 	(5.00f)
#define SR04_TIME_2_DISTANCE_M (0.000170f)

#define SR04_CONVERSION_INTERVAL 	50000 /* 50ms for one sonar */

#define _MF_WINDOW_SIZE 3 ///< window size for median filter on sonar range


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

using namespace DriverFramework;

int static cmp(const void *a, const void *b)
{
	return (*(float *)a - * (float *)b);
}

class HC_SR04 : public device::CDev
{
public:
	HC_SR04(enum Rotation rotation, bool enable_median_filter);
	virtual ~HC_SR04();

	virtual int 		init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:

#if defined(HC_SR04_CAPTURE_DEBUG)

	typedef struct timer_inf_t {
		uint32_t chan_index;
		hrt_abstime edge_time;
		uint32_t edge_state;
		uint32_t overflow;
	} timer_inf_t;

	volatile timer_inf_t _edges[16] = {{0, 0, 0, 0}}; // Must be a power of 2
	volatile int 		 _edge_index = 0;

	volatile hrt_abstime _distance_time[16] = {0}; // Must be a power of 2
	volatile int 		 _distance_time_index_on = 0;
	volatile int 		 _distance_time_index_off = 0;
#else
	volatile hrt_abstime _distance_time = 0;
#endif

	bool				_should_exit = false;

	float				_min_distance;
	float				_max_distance;
	float 				_mf_window[_MF_WINDOW_SIZE] = {};
	float				_mf_window_sorted[_MF_WINDOW_SIZE] = {};
	int 				_mf_cycle_counter;
	work_s				_work = {};
	ringbuffer::RingBuffer	*_reports;
	volatile enum {
		Uninitialized = 0,
		Initialized,
		WatingRising,
		WatingFalling,
	} _sensor_state;

	int					_measure_ticks;
	volatile bool				_sensor_data_available;
	int					_class_instance;
	int					_orb_class_instance;

	bool			_enable_median_filter;

	enum Rotation 		_rotation;
	orb_advert_t		_sensor_info_pub;
	orb_advert_t		_distance_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;


	std::vector<float>
	_latest_sonar_measurements; /* vector to store latest sonar measurements in before writing to report */

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults MB12XX_MIN_DISTANCE
	* and MB12XX_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);

	static void capture_trampoline(void *context, uint32_t chan_index,
				       hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

	void capture_callback(uint32_t chan_index,
			      hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

	float median_filter(float value);


};


/*
 * Driver 'main' command.
 */
extern "C"  __EXPORT int hc_sr04_main(int argc, char *argv[]);

HC_SR04::HC_SR04(enum Rotation rotation, bool enable_median_filter) :
	CDev("HC_SR04", SR04_DEVICE_PATH, 0),
	_min_distance(SR04_MIN_DISTANCE),
	_max_distance(SR04_MAX_DISTANCE),
	_mf_cycle_counter(0),
	_reports(nullptr),
	_sensor_state(Uninitialized),
	_measure_ticks(0),
	_sensor_data_available(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_enable_median_filter(enable_median_filter),
	_rotation(rotation),
	_sensor_info_pub(nullptr),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "hc_sr04_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hc_sr04_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "hc_sr04_buffer_overflows"))

{
	/* enable debug() calls */
	_debug_enabled = false;
}

HC_SR04::~HC_SR04()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
HC_SR04::init()
{
	/* do init (and probe) first */
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	int fd_fmu;
	int fd_pwm;

	fd_fmu = ::open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd_fmu == -1) {
		PX4_WARN("FMU: px4_open fail");
		return PX4_ERROR;
	}

	fd_pwm = ::open(PWM_OUTPUT0_DEVICE_PATH, O_RDWR);

	if (fd_pwm == -1) {
		PX4_WARN("PMW: px4_open fail");
		return PX4_ERROR;
	}

	unsigned capture_count = 0;

	// set PWM rate for the sonar to 20Hz
	if (::ioctl(fd_pwm, PWM_SERVO_SET_UPDATE_RATE, 20) != OK) {
		PX4_ERR("PWM_SERVO_SET_UPDATE_RATE fail");
		return PX4_ERROR;
	}

	unsigned group = 1;
	uint32_t group_mask;

	// Get the group mask
	if (::ioctl(fd_pwm, PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask) != OK) {
		PX4_ERR("PWM_SERVO_GET_RATEGROUP group %u fail", group);
		return PX4_ERROR;
	}

	// Apply group mask
	if (::ioctl(fd_pwm, PWM_SERVO_SET_SELECT_UPDATE_RATE, group_mask) != OK) {
		PX4_ERR("PWM_SERVO_SET_SELECT_UPDATE_RATE fail");
		return PX4_ERROR;
	}

	if (::ioctl(fd_pwm, PWM_SERVO_ARM, 0) != OK) {
		PX4_ERR("PWM_SERVO_ARM fail");
		return PX4_ERROR;
	}

	// Set pulsewidth of 10ms specific for this sonar sensor
	if (::ioctl(fd_pwm, PWM_SERVO_SET(group), 10) != OK) {
		PX4_ERR("PWM_SERVO_SET group %u fail", group);
		return PX4_ERROR;
	}

	input_capture_config_t cap_config;
	cap_config.channel = 2;
	cap_config.filter = 0xf;
	cap_config.edge = Both;
	cap_config.callback = &HC_SR04::capture_trampoline;
	cap_config.context = this; // pass reference to this class

	if (::ioctl(fd_fmu, INPUT_CAP_GET_COUNT, (unsigned long)&capture_count) != 0) {
		PX4_ERR("Not in a capture mode");
		return PX4_ERROR;
	}

	if (::ioctl(fd_fmu, INPUT_CAP_SET_CALLBACK, (unsigned long)&cap_config) != 0) {
		PX4_ERR("INPUT_CAP_SET_CALLBACK fail for chan %u", cap_config.channel);
		return PX4_ERROR;
	}

	if (::ioctl(fd_fmu, INPUT_CAP_SET, (unsigned long)&cap_config) != 0) {
		PX4_ERR("INPUT_CAP_SET fail");
		return PX4_ERROR;
	}

	/* sensor is ok, but we don't really know if it is within range */
	_sensor_state = Initialized;

	// start driver
	start();

	return OK;
}

int
HC_SR04::probe()
{
	return (OK);
}

void
HC_SR04::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
HC_SR04::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
HC_SR04::get_minimum_distance()
{
	return _min_distance;
}

float
HC_SR04::get_maximum_distance()
{
	return _max_distance;
}

// TODO: review this part since it is not compatible with the new architecture
int
HC_SR04::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(SR04_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(SR04_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case RANGEFINDERIOCSETMINIUMDISTANCE: {
			set_minimum_distance(*(float *)arg);
			return 0;
		}
		break;

	case RANGEFINDERIOCSETMAXIUMDISTANCE: {
			set_maximum_distance(*(float *)arg);
			return 0;
		}
		break;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

int
HC_SR04::measure()
{
	int ret = -EAGAIN;

	if (_sensor_state != Uninitialized) {

		hrt_abstime wait = hrt_absolute_time() + (2 * SR04_CONVERSION_INTERVAL);

		while (ret == -EAGAIN) {
			if (wait < hrt_absolute_time()) {
				ret = -ENXIO;

			} else if (_sensor_state == Initialized) {
				usleep(SR04_CONVERSION_INTERVAL / 2);

			} else {
				ret = PX4_OK;
			}
		}
	}

	return ret;
}

float
HC_SR04::median_filter(float value)
{
	/* TODO: replace with ring buffer */
	_mf_window[(_mf_cycle_counter + 1) % _MF_WINDOW_SIZE] = value;

	for (int i = 0; i < _MF_WINDOW_SIZE; ++i) {
		_mf_window_sorted[i] = _mf_window[i];
	}

	qsort(_mf_window_sorted, _MF_WINDOW_SIZE, sizeof(float), cmp);

	_mf_cycle_counter++;

	if (_mf_window_sorted[(_MF_WINDOW_SIZE / 2)] < get_maximum_distance()) {
		return _mf_window_sorted[(_MF_WINDOW_SIZE / 2)];

	} else {
		return get_maximum_distance();
	}
}

void
HC_SR04::start()
{
	/* reset the report ring */
	_reports->flush();

	_should_exit = false;

	measure();  /* Wait on measurement */

	/* notify about state change */
	struct subsystem_info_s info = {};
	info.timestamp = hrt_absolute_time();
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = SUBSYSTEM_TYPE_RANGEFINDER;

	if (_sensor_info_pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), _sensor_info_pub, &info);

	} else {
		_sensor_info_pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}

	work_queue(HPWORK, &_work, (worker_t)&HC_SR04::cycle_trampoline, this, 0);
}

void
HC_SR04::stop()
{
	/* unadvertise publishing topics */
	orb_unadvertise(_sensor_info_pub);
	orb_unadvertise(_distance_sensor_topic);

	DevHandle h_fmu;
	DevHandle h_pwm;

	DevMgr::getHandle(PX4FMU_DEVICE_PATH, h_fmu);

	if (!h_fmu.isValid()) {
		PX4_WARN("FMU: px4_open fail\n");
	}

	DevMgr::getHandle(PWM_OUTPUT0_DEVICE_PATH, h_pwm);

	if (!h_pwm.isValid()) {
		PX4_WARN("PMW: px4_open fail\n");
	}

	h_pwm.ioctl(PWM_SERVO_SET_UPDATE_RATE, 50);
	unsigned group = 1;
	uint32_t group_mask;

	if (h_pwm.ioctl(PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask) != OK) {
		PX4_ERR("pwm servo get rategroup fail");
	}

	if (h_pwm.ioctl(PWM_SERVO_SET_SELECT_UPDATE_RATE, group_mask) != OK) {
		PX4_ERR("pwm servo set select update rate fail");
	}

	input_capture_config_t cap_config;
	cap_config.channel = 2;
	cap_config.filter = 0;
	cap_config.edge = Disabled;
	cap_config.callback = nullptr;
	cap_config.context = nullptr;

	h_fmu.ioctl(INPUT_CAP_SET_CALLBACK, (unsigned long)&cap_config);
	_should_exit = true;
	work_cancel(HPWORK, &_work);
}

void
HC_SR04::cycle_trampoline(void *arg)
{
	HC_SR04 *dev = (HC_SR04 *)arg;

	dev->cycle();

}

void
HC_SR04::cycle()
{
	if (_should_exit) {
		// exit
		return;
	}

	irqstate_t flags = px4_enter_critical_section();
	bool sensor_data_available = _sensor_data_available;

	// reset flag
	if (_sensor_data_available) {
		_sensor_data_available = false;
	}

	float distance = _distance_time * SR04_TIME_2_DISTANCE_M;
	px4_leave_critical_section(flags);

	if (!sensor_data_available) {
		work_queue(HPWORK, &_work, (worker_t)&HC_SR04::cycle_trampoline, this, USEC2TICK(SR04_CONVERSION_INTERVAL));
		return;
	}

	struct distance_sensor_s report = {};

	report.timestamp = hrt_absolute_time();

	report.min_distance = get_minimum_distance();

	report.max_distance = get_maximum_distance();

	if (_enable_median_filter) {
		report.current_distance = median_filter(distance);

	} else {
		report.current_distance = distance;
	}

	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = _rotation;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);

	} else {
		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &report,
					 &_orb_class_instance, ORB_PRIO_LOW);
	}

	work_queue(HPWORK, &_work, (worker_t)&HC_SR04::cycle_trampoline, this, USEC2TICK(SR04_CONVERSION_INTERVAL));
}

void
HC_SR04::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

void HC_SR04::capture_trampoline(void *context, uint32_t chan_index,
				 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	HC_SR04 *dev = reinterpret_cast<HC_SR04 *>(context);
	dev->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void HC_SR04::capture_callback(uint32_t chan_index,
			       hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	static hrt_abstime raising_time = 0;
	static hrt_abstime falling_time = 0;

#if defined(HC_SR04_CAPTURE_DEBUG)
	_edges[_edge_index].chan_index = chan_index;
	_edges[_edge_index].edge_time  = edge_time;
	_edges[_edge_index].edge_state =  edge_state;
	_edges[_edge_index++].overflow = overflow;
	_edge_index &= arraySize(_edges) - 1;
#endif

	if (edge_state == 1) {
		_sensor_state = WatingFalling;
		raising_time = edge_time;

	} else {
		_sensor_state = WatingRising;
		falling_time = edge_time;

#if defined(HC_SR04_CAPTURE_DEBUG)
		_distance_time[_distance_time_index_on++] = falling_time - raising_time;
		_distance_time_index_on &= arraySize(_distance_time) - 1;
#else
		_distance_time = falling_time - raising_time;
#endif
		_sensor_data_available = true;
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace  hc_sr04
{

HC_SR04	*g_dev;

void	start(enum Rotation rotation, bool enable_median_filter);
void	stop();
// void	test();
void	info();

/**
 * Start the driver.
 */
void
start(enum Rotation rotation, bool enable_median_filter)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		exit(1);
	}

	/* create the driver */
	g_dev = new HC_SR04(rotation, enable_median_filter);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	exit(1);
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		exit(1);
	}

	exit(0);
}

// TODO: need reimplementation
/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
// void
// test()
// {
// 	struct distance_sensor_s report;
// 	ssize_t sz;
// 	int ret;
//
// 	int fd = open(SR04_DEVICE_PATH, O_RDONLY);
//
// 	if (fd < 0) {
// 		PX4_ERR("%s open failed (try 'hc_sr04 start' if the driver is not running", SR04_DEVICE_PATH);
// 		exit(1);
// 	}
//
// 	/* do a simple demand read */
// 	sz = read(fd, &report, sizeof(report));
//
// 	if (sz != sizeof(report)) {
// 		PX4_ERR("immediate read failed");
// 		exit(1);
// 	}
//
// 	PX4_INFO("single read");
// 	PX4_INFO("measurement: %0.2f", (double)report.current_distance);
// 	PX4_INFO("time:        %lld", report.timestamp);
//
// 	/* start the sensor polling at 2Hz */
// 	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 20)) {
// 		PX4_ERR("failed to set 20Hz poll rate");
// 		exit(1);
// 	}
//
// 	/* read the sensor 5x and report each value */
// 	for (unsigned i = 0; i < 5; i++) {
// 		struct pollfd fds;
//
// 		/* wait for data to be ready */
// 		fds.fd = fd;
// 		fds.events = POLLIN;
// 		ret = poll(&fds, 1, 2000);
//
// 		if (ret != 1) {
// 			PX4_ERR("timed out waiting for sensor data");
// 			exit(1);
// 		}
//
// 		/* now go get it */
// 		sz = read(fd, &report, sizeof(report));
//
// 		if (sz != sizeof(report)) {
// 			PX4_ERR("periodic read failed");
// 			exit(1);
// 		}
//
// 		PX4_INFO("periodic read %u", i);
//
// 		/* Print the sonar rangefinder report sonar distance vector */
// 		PX4_INFO("measurement: %0.3f", (double)report.current_distance);
//
// 		PX4_INFO("time:        %lld", report.timestamp);
// 	}
//
// 	/* reset the sensor polling to default rate */
// 	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
// 		PX4_ERR("failed to set default poll rate");
// 		exit(1);;
// 	}
//
// 	PX4_INFO("PASS");
// 	exit(0);
// }


/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} /* namespace */


int
hc_sr04_main(int argc, char *argv[])
{

	int ch;
	bool enable_median_filter = false;
	enum Rotation rotation = ROTATION_NONE;
	int myoptind = 1;
	const char *myoptarg = NULL;


	while ((ch = px4_getopt(argc, argv, "R:m", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'm':
			enable_median_filter = true;
			break;

		default:
			PX4_WARN("missing rotation information");
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		hc_sr04::start(rotation, enable_median_filter);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(verb, "stop")) {
		hc_sr04::stop();
	}

	/*
	 * Test the driver/device.
	 */
	// if (!strcmp(verb, "test")) {
	// 	hc_sr04::test();
	// }

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		hc_sr04::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test' or 'info'");
	exit(1);
}
