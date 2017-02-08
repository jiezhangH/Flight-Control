/****************************************************************************
 *
 *   Copyright (c) YUNEEC Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file tap_esc_rgbled.cpp
 *
 * Driver for the RGB LED controller.
 * Adaped from drivers/rgbled_pwm
 *
 * @author Haohua Chang <872561481@qq.com>
 * @author Simone Guscetti <simone@px4.io>
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <px4_workqueue.h>
#include <drivers/drv_hrt.h>
#include <drivers/tap_esc/drv_tap_esc.h>
#include <drivers/drv_rgbled.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_rgbled.h>
#include <drivers/device/device.h>
#include <systemlib/err.h>

#include <px4_getopt.h>
#include <uORB/uORB.h>
#include <uORB/topics/leds.h>
#include <uORB/topics/led_event.h>

#define RGBLED_ONTIME 120
#define RGBLED_OFFTIME 120

// if any event are not scheduled run with a frequency of 10Hz
#define TAP_RGBLED_MIN_INTERVAL_US 100000

class TapEscRGBLED : public device::CDev
{
public:
	TapEscRGBLED();
	virtual ~TapEscRGBLED();


	virtual int		init();
	void stop();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
	void set_number_of_leds(uint8_t n_leds);

private:

	struct work_s		_work = {};

	rgbled_mode_t		_mode;
	rgbled_pattern_t	_pattern;

	bool			_enable;
	bool 			_led_was_enabled;
	float			_brightness;

	volatile bool 	_task_is_running;
	volatile bool 	_task_should_exit;
	int				_led_interval;
	bool			_should_run;
	uint32_t		_counter;
	uint16_t		_led_color;
	orb_advert_t	_leds_pub = nullptr;
	int				_led_event_sub = -1;
	uint8_t 		_n_leds = TAP_ESC_MAX_MOTOR_NUM; // One led for motor
	uint8_t 		_prio = 2;
	uint32_t		_computed_transition[3] = {0, 0, 0}; // just for the high and medium prio tasks

	struct led_event_s _events_prio[3] = {{}, {}, {}};

	void set_color(rgbled_color_t ledcolor, struct led_event_s &led_event_prio);
	void set_mode(rgbled_mode_t mode, struct led_event_s &led_event_prio);
	void set_pattern(rgbled_pattern_t *pattern);
	void set_mode_and_color(rgbled_mode_and_color_t *mode_color);

	int active_events();

	static void led_trampoline(void *arg);
	void led();

	int send_led_enable(bool enable);
	int send_led_rgb();
	int get(bool &on, bool &powersave, uint16_t &rgb);
	int get_mode_ticks(rgbled_mode_t mode);
};

extern "C" __EXPORT int tap_esc_rgbled_main(int argc, char *argv[]);

namespace
{
TapEscRGBLED *tap_esc_rgbled = nullptr;
}

void tap_esc_rgbled_usage();

TapEscRGBLED::TapEscRGBLED() :
	CDev("tap_esc_rgbled", RGBLED0_DEVICE_PATH),
	_mode(RGBLED_MODE_OFF),
	_enable(false),
	_brightness(1.0f),
	_led_interval(0),
	_should_run(false),
	_counter(0)
{
	memset(&_pattern, 0, sizeof(_pattern));
}

TapEscRGBLED::~TapEscRGBLED()
{

}

int
TapEscRGBLED::init()
{
	/* switch off LED on start */
	CDev::init();
	send_led_enable(false);
	send_led_rgb();
	_task_is_running = true;
	_task_should_exit = false;
	// make sure the enable field is set 0
	_events_prio[0].enabled = 0;
	_events_prio[1].enabled = 0;
	//schedule work call with a fequency of 10Hz
	return 	work_queue(LPWORK, &_work, (worker_t)&TapEscRGBLED::led_trampoline, this,
			   USEC2TICK(TAP_RGBLED_MIN_INTERVAL_US));
}


int
TapEscRGBLED::info()
{
	int ret;
	bool on, powersave;
	uint16_t rgb;

	ret = get(on, powersave, rgb);

	if (ret == OK) {
		/* we don't care about power-save mode */
		DEVICE_LOG("state: %s", on ? "ON" : "OFF");
		DEVICE_LOG("rgb: 0x%04x", rgb);

	} else {
		PX4_WARN("failed to read led");
	}

	return ret;
}
int
TapEscRGBLED::probe()
{
	return (OK);
}
int
TapEscRGBLED::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case RGBLED_SET_COLOR:
		/* set the specified color name */
		set_color((rgbled_color_t)arg, _events_prio[2]);
		send_led_rgb();
		return OK;

	case RGBLED_SET_MODE:
		/* set the specified mode */
		set_mode((rgbled_mode_t)arg, _events_prio[2]);
		return OK;

	case RGBLED_SET_MODE_AND_COLOR:
		set_mode_and_color((rgbled_mode_and_color_t *)arg);
		return OK;

	case RGBLED_SET_PATTERN:
		/* set a special pattern */
		set_pattern((rgbled_pattern_t *)arg);
		return OK;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}


void
TapEscRGBLED::led_trampoline(void *arg)
{
	TapEscRGBLED *rgbl = reinterpret_cast<TapEscRGBLED *>(arg);

	rgbl->led();
}

/**
 * Main loop function
 */
void
TapEscRGBLED::led()
{
	bool event_expired = false;

	if (_task_should_exit) {
		_task_is_running = false;
		return;
	}

	// reset event
	if (_computed_transition[0] > _events_prio[0].duration && _events_prio[0].duration != 0
	    && _events_prio[0].enabled) {
		_computed_transition[0] = 0;
		_events_prio[0].enabled = 0;
		event_expired = true;
	}

	if (_computed_transition[1] > _events_prio[1].duration && _events_prio[1].duration != 0
	    && _events_prio[1].enabled) {
		_computed_transition[1] = 0;
		_events_prio[1].enabled = 0;
		event_expired = true;
	}

	if (event_expired) {
		_led_was_enabled = false;
		_counter = 0;
		set_mode(RGBLED_MODE_BREATHE, _events_prio[2]);
	}

	// subscribe to led_event topic
	if (_led_event_sub < 0) {
		_led_event_sub = orb_subscribe(ORB_ID(led_event));
	}

	bool updated;
	orb_check(_led_event_sub, &updated);

	if (updated) {
		if (active_events() != 0) {
			_counter = 0;
			_led_was_enabled = false;
		}

		_computed_transition[1] = 0;
		_mode = RGBLED_MODE_OFF;
		orb_copy(ORB_ID(led_event), _led_event_sub, &_events_prio[1]);
	}

	bool led_enabled;
	int priority = active_events();
	rgbled_mode_t mode = (rgbled_mode_t)_events_prio[priority].mode[0];

	switch (mode) {
	case RGBLED_MODE_BLINK_SLOW:
	case RGBLED_MODE_BLINK_NORMAL:
	case RGBLED_MODE_BLINK_FAST:
		led_enabled = (_counter % get_mode_ticks(mode)) == (_counter % (2 * get_mode_ticks(mode)));

		if (led_enabled != _led_was_enabled) {
			send_led_enable(led_enabled);
			_computed_transition[priority]++;
			_led_was_enabled = led_enabled;
		}

		break;

	case RGBLED_MODE_BREATHE:
	case RGBLED_MODE_ON:
		if (_counter == 0) {
			send_led_enable(true);
		}

	case RGBLED_MODE_OFF:
		if (_counter == 0) {
			send_led_enable(false);
		}

	default:
		break;
	}

	_counter++;

	// prevent overflow TODO:check if needed?
	if (_counter > 9999) {
		_counter = 0;
	}

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&TapEscRGBLED::led_trampoline, this, USEC2TICK(TAP_RGBLED_MIN_INTERVAL_US));
}

/**
 * Parse color constant and set _led_color value
 */
void
TapEscRGBLED::set_color(rgbled_color_t color, struct led_event_s &led_event_prio)
{
	uint16_t led_color = 0;

	switch (color) {
	case RGBLED_COLOR_OFF:
		led_color = 0;
		break;

	case RGBLED_COLOR_RED:
		led_color = RUN_RED_LED_ON_MASK;
		break;

	case RGBLED_COLOR_YELLOW:

	// amber case handaled as yellow
	case RGBLED_COLOR_AMBER:
		led_color = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
		break;

	case RGBLED_COLOR_PURPLE:
		led_color = RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
		break;

	case RGBLED_COLOR_GREEN:
		led_color = RUN_GREEN_LED_ON_MASK;
		break;

	case RGBLED_COLOR_BLUE:
		led_color = RUN_BLUE_LED_ON_MASK;
		break;

	case RGBLED_COLOR_WHITE:
		led_color = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
		break;

	default:
		PX4_WARN("color not supported");
		break;
	}

	// set color in the event structure
	for (size_t i = 0; i < _n_leds; i++) {
		led_event_prio.color[i] = led_color;
	}

}

/**
 * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
 */
void
TapEscRGBLED::set_mode(rgbled_mode_t mode, struct led_event_s &led_event_prio)
{
	for (size_t i = 0; i < _n_leds; i++) {
		led_event_prio.mode[i] = mode;
	}
}

int TapEscRGBLED::get_mode_ticks(rgbled_mode_t mode)
{
	int ticks = 1;

	switch (mode) {
	case RGBLED_MODE_BLINK_SLOW:
		ticks = 20;
		break;

	case RGBLED_MODE_BLINK_NORMAL:
		ticks = 5;
		break;

	case RGBLED_MODE_BLINK_FAST:
		ticks = 1;
		break;

	default:
		ticks = 1;
		break;
	}

	return ticks;
}

void TapEscRGBLED::set_mode_and_color(rgbled_mode_and_color_t *mode_color)
{
	uint8_t index = mode_color->prio > 2 ? 2 : mode_color->prio;
	_events_prio[index].enabled = mode_color->enabled;
	_events_prio[index].timestamp = hrt_absolute_time();
	_events_prio[index].duration = mode_color->duration;
	set_color(mode_color->color, _events_prio[index]);
	set_mode(mode_color->mode, _events_prio[index]);
	_counter = 0;
	_led_was_enabled = false;
	_computed_transition[index] = 0;
}

int TapEscRGBLED::active_events()
{
	int ret = 2;

	if (_events_prio[0].enabled) {
		ret = 0;

	} else if (_events_prio[1].enabled) {
		ret = 1;
	}

	return ret;
}

/**
 * Set pattern for PATTERN mode, but don't change current mode
 */
void
TapEscRGBLED::set_pattern(rgbled_pattern_t *pattern)
{
	memcpy(&_pattern, pattern, sizeof(rgbled_pattern_t));
}

/**
 * Sent ENABLE flag to LED driver
 */
int
TapEscRGBLED::send_led_enable(bool enable)
{
	_enable = enable;
	send_led_rgb();
	return (OK);
}

/**
 * Publish LEDs color over uORB to the ESC driver
 */
int
TapEscRGBLED::send_led_rgb()
{
	struct leds_s leds = {};

	if (_enable) {
		int index = active_events();

		for (uint8_t i = 0; i < _n_leds; i++) {
			leds.color[i] = _events_prio[index].color[i];
		}
	}

	// publish leds msg
	if (_leds_pub != nullptr) {
		orb_publish(ORB_ID(leds), _leds_pub, &leds);

	} else {
		_leds_pub =  orb_advertise(ORB_ID(leds), &leds);
	}

	return (OK);
}

int
TapEscRGBLED::get(bool &on, bool &powersave, uint16_t &rgb)
{
	powersave = OK;
	on = _enable;
	rgb = _led_color;
	return OK;
}

void TapEscRGBLED::stop()
{
	if (!_task_is_running) {
		return;
	}

	_task_should_exit = true;
	// Wait for task to exit
	int i = 0;

	do {
		/* wait up to 3s */
		usleep(100000);

	} while (_task_is_running && ++i < 30);

	if (i == 30) {
		PX4_ERR("failed to stop");
	}
}

void TapEscRGBLED::set_number_of_leds(uint8_t n_leds)
{
	_n_leds = n_leds;
}

void
tap_esc_rgbled_usage()
{
	PX4_WARN("missing command: tap_esc_rgbled_usage try 'start', 'test', 'info', 'off', 'stop'");
}

int
tap_esc_rgbled_main(int argc, char *argv[])
{
	int ch;

	/* jump over start/off/etc and look at options first */
	// TODO migrate this to px4_getopt
	while ((ch = getopt(argc, argv, "m:")) != EOF) {
		switch (ch) {
		case 'm':
			// TODO: read value
			break;

		default:
			tap_esc_rgbled_usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		tap_esc_rgbled_usage();
		exit(1);
	}

	const char *verb = argv[optind];

	int fd;
	int ret;

	if (!strcmp(verb, "start")) {
		if (tap_esc_rgbled != nullptr) {
			PX4_WARN("already started");
		}

		if (tap_esc_rgbled == nullptr) {
			tap_esc_rgbled = new TapEscRGBLED();
			// TODO: get this information from getopt
			tap_esc_rgbled->set_number_of_leds(6);

			if (tap_esc_rgbled == nullptr) {
				PX4_ERR("new failed");
			}

			if (OK != tap_esc_rgbled->init()) {
				delete tap_esc_rgbled;
				tap_esc_rgbled = nullptr;
				PX4_ERR("init failed");
			}
		}

		exit(0);
	}

	/* need the driver past this point */
	if (tap_esc_rgbled == nullptr) {
		PX4_WARN("not started");
		tap_esc_rgbled_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		fd = open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			PX4_ERR("Unable to open " RGBLED0_DEVICE_PATH);
		}

		rgbled_pattern_t pattern = { {RGBLED_COLOR_RED, RGBLED_COLOR_GREEN, RGBLED_COLOR_BLUE, RGBLED_COLOR_WHITE, RGBLED_COLOR_AMBER, RGBLED_COLOR_OFF, RGBLED_COLOR_OFF},
			{500, 500, 500, 500, 500, 1000, 0 }	// "0" indicates end of pattern
		};

		ret = ioctl(fd, RGBLED_SET_PATTERN, (unsigned long)&pattern);
		ret = ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_PATTERN);

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "info")) {
		tap_esc_rgbled->info();
		exit(0);
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		fd = open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			PX4_ERR("Unable to open %s", RGBLED0_DEVICE_PATH);
		}

		ret = ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
		close(fd);

		/* delete the rgbled object if stop was requested. In addition,turn off the LED. */
		if (!strcmp(verb, "stop")) {
			tap_esc_rgbled->stop();
			delete tap_esc_rgbled;
			tap_esc_rgbled = nullptr;
			exit(0);
		}

		exit(ret);
	}

	tap_esc_rgbled_usage();
	exit(0);
}
