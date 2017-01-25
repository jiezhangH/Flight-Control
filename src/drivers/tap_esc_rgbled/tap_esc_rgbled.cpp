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

#include <nuttx/wqueue.h>
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
#include <uORB/topics/tap_leds.h>

#define RGBLED_ONTIME 120
#define RGBLED_OFFTIME 120

class TAP_ESC_RGBLED : public device::CDev
{
public:
	TAP_ESC_RGBLED();
	virtual ~TAP_ESC_RGBLED();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
	void set_number_of_leds(uint8_t n_leds);

private:

	work_s				_work;

	rgbled_mode_t		_mode;
	rgbled_pattern_t	_pattern;

	bool			_enable;
	float			_brightness;

	bool			_running;
	int				_led_interval;
	bool			_should_run;
	int				_counter;
	uint16_t		_led_color;
	orb_advert_t	_tap_leds_pub = nullptr;
	uint8_t 		_n_leds = TAP_ESC_MAX_MOTOR_NUM; // One led for motor


	void 			set_color(rgbled_color_t ledcolor);
	void			set_mode(rgbled_mode_t mode);
	void			set_pattern(rgbled_pattern_t *pattern);

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_enable(bool enable);
	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint16_t &rgb);
};

extern "C" __EXPORT int tap_esc_rgbled_main(int argc, char *argv[]);

namespace
{
TAP_ESC_RGBLED *tap_esc_rgbled = nullptr;
}

void tap_esc_rgbled_usage();

TAP_ESC_RGBLED::TAP_ESC_RGBLED() :
	CDev("tap_esc_rgbled", RGBLED0_DEVICE_PATH),
	_mode(RGBLED_MODE_OFF),
	_enable(false),
	_brightness(1.0f),
	_running(false),
	_led_interval(0),
	_should_run(false),
	_counter(0)
{
	memset(&_work, 0, sizeof(_work));
	memset(&_pattern, 0, sizeof(_pattern));
}

TAP_ESC_RGBLED::~TAP_ESC_RGBLED()
{

}

int
TAP_ESC_RGBLED::init()
{
	/* switch off LED on start */
	CDev::init();
	send_led_enable(false);
	send_led_rgb();
	return OK;
}


int
TAP_ESC_RGBLED::info()
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
TAP_ESC_RGBLED::probe()
{
	return (OK);
}
int
TAP_ESC_RGBLED::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case RGBLED_SET_COLOR:
		/* set the specified color name */
		set_color((rgbled_color_t)arg);
		send_led_rgb();
		return OK;

	case RGBLED_SET_MODE:
		/* set the specified mode */
		set_mode((rgbled_mode_t)arg);
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
TAP_ESC_RGBLED::led_trampoline(void *arg)
{
	TAP_ESC_RGBLED *rgbl = reinterpret_cast<TAP_ESC_RGBLED *>(arg);

	rgbl->led();
}

/**
 * Main loop function
 */
void
TAP_ESC_RGBLED::led()
{
	if (!_should_run) {
		_running = false;
		return;
	}

	switch (_mode) {
	case RGBLED_MODE_BLINK_SLOW:
	case RGBLED_MODE_BLINK_NORMAL:
	case RGBLED_MODE_BLINK_FAST:
		if (_counter >= 2) {
			_counter = 0;
		}

		send_led_enable(_counter == 0);

		break;

	case RGBLED_MODE_BREATHE:

		if (_counter >= 62) {
			_counter = 0;
		}

		int n;

		if (_counter < 32) {
			n = _counter;

		} else {
			n = 62 - _counter;
		}

		_brightness = n * n / (31.0f * 31.0f);
//		send_led_rgb();
		break;

	case RGBLED_MODE_PATTERN:

		/* don't run out of the pattern array and stop if the next frame is 0 */
		if (_counter >= RGBLED_PATTERN_LENGTH || _pattern.duration[_counter] <= 0) {
			_counter = 0;
		}

		set_color(_pattern.color[_counter]);
		send_led_rgb();
		_led_interval = _pattern.duration[_counter];
		break;

	default:
		break;
	}

	_counter++;

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&TAP_ESC_RGBLED::led_trampoline, this, _led_interval);
}

/**
 * Parse color constant and set _led_color value
 */
void
TAP_ESC_RGBLED::set_color(rgbled_color_t color)
{
	switch (color) {
	case RGBLED_COLOR_OFF:
		_led_color = 0;
		break;

	case RGBLED_COLOR_RED:
		_led_color = RUN_RED_LED_ON_MASK;
		break;

	case RGBLED_COLOR_YELLOW:

	// amber case handaled as yellow
	case RGBLED_COLOR_AMBER:
		_led_color = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
		break;

	case RGBLED_COLOR_PURPLE:
		_led_color = RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
		break;

	case RGBLED_COLOR_GREEN:
		_led_color = RUN_GREEN_LED_ON_MASK;
		break;

	case RGBLED_COLOR_BLUE:
		_led_color = RUN_BLUE_LED_ON_MASK;
		break;

	case RGBLED_COLOR_WHITE:
		_led_color = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
		break;

	default:
		PX4_WARN("color not supported");
		break;
	}

}

/**
 * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
 */
void
TAP_ESC_RGBLED::set_mode(rgbled_mode_t mode)
{
	if (mode != _mode) {
		_mode = mode;

		switch (mode) {
		case RGBLED_MODE_OFF:
			_should_run = false;
			send_led_enable(false);
			break;

		case RGBLED_MODE_ON:
			_brightness = 1.0f;
			send_led_rgb();
			send_led_enable(true);
			break;

		case RGBLED_MODE_BLINK_SLOW:
			_should_run = true;
			_counter = 0;
			_led_interval = 2000;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BLINK_NORMAL:
			_should_run = true;
			_counter = 0;
			_led_interval = 500;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BLINK_FAST:
			_should_run = true;
			_counter = 0;
			_led_interval = 100;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BREATHE:
			_should_run = true;
			_counter = 0;
			_led_interval = 25;
			send_led_enable(true);
			break;

		case RGBLED_MODE_PATTERN:
			_should_run = true;
			_counter = 0;
			_brightness = 1.0f;
			send_led_enable(true);
			break;

		default:
			PX4_WARN("mode unknown");
			break;
		}

		/* if it should run now, start the workq */
		if (_should_run && !_running) {
			_running = true;
			work_queue(LPWORK, &_work, (worker_t)&TAP_ESC_RGBLED::led_trampoline, this, 1);
		}

	}
}

/**
 * Set pattern for PATTERN mode, but don't change current mode
 */
void
TAP_ESC_RGBLED::set_pattern(rgbled_pattern_t *pattern)
{
	memcpy(&_pattern, pattern, sizeof(rgbled_pattern_t));
}

/**
 * Sent ENABLE flag to LED driver
 */
int
TAP_ESC_RGBLED::send_led_enable(bool enable)
{
	_enable = enable;
	send_led_rgb();
	return (OK);
}

/**
 * Publish LEDs color over uORB to the ESC driver
 */
int
TAP_ESC_RGBLED::send_led_rgb()
{
	struct tap_leds_s leds;
	memset(&leds, 0, sizeof(leds));

	if (_enable) {
		for (uint8_t i = 0; i < _n_leds; i++) {
			leds.color[i] = _led_color;
		}
	}

	// publish tap_leds msg
	if (_tap_leds_pub != nullptr) {
		orb_publish(ORB_ID(tap_leds), _tap_leds_pub, &leds);

	} else {
		_tap_leds_pub =  orb_advertise(ORB_ID(tap_leds), &leds);
	}

	return (OK);
}

int
TAP_ESC_RGBLED::get(bool &on, bool &powersave, uint16_t &rgb)
{
	powersave = OK;
	on = _enable;
	rgb = _led_color;
	return OK;
}

void TAP_ESC_RGBLED::set_number_of_leds(uint8_t n_leds)
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
			tap_esc_rgbled = new TAP_ESC_RGBLED();
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
			delete tap_esc_rgbled;
			tap_esc_rgbled = nullptr;
			exit(0);
		}

		exit(ret);
	}

	tap_esc_rgbled_usage();
	exit(0);
}
