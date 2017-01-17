/****************************************************************************
 *
 *   Copyright (c) YUNEEC Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file tap_esc_rgbled.cpp
 *
 * Driver for the RGB LED controller connected via ttyS2 to esc.
 *
 * @author Haohua Chang <872561481@qq.com>
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
#include <drivers/drv_tap_esc.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_rgbled.h>
#include <drivers/device/device.h>
#include <systemlib/err.h>


#define RGBLED_ONTIME 120
#define RGBLED_OFFTIME 120
#define MOTOR_NUMBER 6

class TAP_ESC_RGBLED : public device::CDev
{
public:
	TAP_ESC_RGBLED();
	virtual ~TAP_ESC_RGBLED();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
	uint16_t 		_get_rgbled_color[MOTOR_NUMBER] = {};

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
	int				_led_id;
	uint16_t 		_set_rgbled_color[MOTOR_NUMBER];


	void 			set_color(rgbled_color_t ledcolor);
	void			set_mode(rgbled_mode_t mode);
	void			set_pattern(rgbled_pattern_t *pattern);

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_enable(bool enable,int led_id);
	int			send_led_rgb(int led_id);
	int			get(bool &on, bool &powersave, uint16_t &rgb, int led_id);
};

extern "C" __EXPORT int tap_esc_rgbled_main(int argc, char *argv[]);

/* for now, we only support one RGBLED */
namespace
{
TAP_ESC_RGBLED *tap_esc_rgbled = nullptr;
}

uint16_t get_tap_esc_rgbled_color(int led_id)
{
	return tap_esc_rgbled->_get_rgbled_color[led_id];
}

void tap_esc_rgbled_usage();

TAP_ESC_RGBLED::TAP_ESC_RGBLED() :
	CDev("tap_esc_rgbled", TAPESC_RGBLED_DEVICE_PATH),
	_get_rgbled_color{},
	_mode(RGBLED_MODE_OFF),
	_enable(false),
	_brightness(1.0f),
	_running(false),
	_led_interval(0),
	_should_run(false),
	_counter(0),
	_led_id(-1),
	_set_rgbled_color{}
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
	for(int i=0; i<MOTOR_NUMBER; i++){
		send_led_enable(false,i);
		send_led_rgb(i);
	}
	return OK;
}


int
TAP_ESC_RGBLED::info()
{
	int ret;
	bool on, powersave;
	uint16_t rgb;

	ret = get(on, powersave, rgb, _led_id);

	if (ret == OK) {
		/* we don't care about power-save mode */
		DEVICE_LOG("state: %s", on ? "ON" : "OFF");
		DEVICE_LOG("rgb: %d, led_id: %d", rgb,_led_id);

	} else {
		warnx("failed to read led");
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

	case RGBLED_SET_ID:
		/* set the tap esc led id*/
		if(arg<MOTOR_NUMBER){
			_led_id = arg;
		}else{
			_led_id = -1;
		}
		return OK;

	case RGBLED_SET_COLOR:
		/* set the specified color name */
		set_color((rgbled_color_t)arg);
		send_led_rgb(_led_id);
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

		for(int i=0; i<MOTOR_NUMBER; i++){
			send_led_enable(_counter == 0,i);
		}

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
		for(int i=0; i<MOTOR_NUMBER; i++){
			set_color(_pattern.color[_counter]);
			send_led_rgb(i);
		}
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
 * Parse color constant and set _r _g _b values
 */
void
TAP_ESC_RGBLED::set_color(rgbled_color_t color)
{

	switch (color) {
	case RGBLED_COLOR_OFF:
		_set_rgbled_color[_led_id] = 0;
		break;

	case RGBLED_COLOR_RED:
		_set_rgbled_color[_led_id] = RUN_RED_LED_ON_MASK;
		break;

	case RGBLED_COLOR_YELLOW:
		_set_rgbled_color[_led_id] = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
		break;

	case RGBLED_COLOR_PURPLE:
		_set_rgbled_color[_led_id] = RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
		break;

	case RGBLED_COLOR_GREEN:
		_set_rgbled_color[_led_id] = RUN_GREEN_LED_ON_MASK;
		break;

	case RGBLED_COLOR_BLUE:
		_set_rgbled_color[_led_id] = RUN_BLUE_LED_ON_MASK;
		break;

	case RGBLED_COLOR_WHITE:
		_set_rgbled_color[_led_id] = RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
		break;
	default:
		warnx("color unknown");
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
			send_led_enable(false,_led_id);
			break;

		case RGBLED_MODE_ON:
			_brightness = 1.0f;
			for(int i=0;i<MOTOR_NUMBER;i++){
				send_led_rgb(i);
				send_led_enable(true,i);
			}
			break;

		case RGBLED_MODE_BLINK_SLOW:
			_should_run = true;
			_counter = 0;
			_led_interval = 2000;
			_brightness = 1.0f;
			for(int i=0;i<MOTOR_NUMBER;i++){
				send_led_rgb(i);
			}
			break;

		case RGBLED_MODE_BLINK_NORMAL:
			_should_run = true;
			_counter = 0;
			_led_interval = 500;
			_brightness = 1.0f;
			for(int i=0;i<MOTOR_NUMBER;i++){
				send_led_rgb(i);
			}
			break;

		case RGBLED_MODE_BLINK_FAST:
			_should_run = true;
			_counter = 0;
			_led_interval = 100;
			_brightness = 1.0f;
			for(int i=0;i<MOTOR_NUMBER;i++){
				send_led_rgb(i);
			}
			break;

		case RGBLED_MODE_BREATHE:
			_should_run = true;
			_counter = 0;
			_led_interval = 25;
			for(int i=0;i<MOTOR_NUMBER;i++){
				send_led_enable(true,i);
			}
			break;

		case RGBLED_MODE_PATTERN:
			_should_run = true;
			_counter = 0;
			_brightness = 1.0f;
			for(int i=0;i<MOTOR_NUMBER;i++){
				send_led_enable(true,i);
			}
			break;

		default:
			warnx("mode unknown");
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
TAP_ESC_RGBLED::send_led_enable(bool enable,int led_id)
{
	_enable = enable;
	send_led_rgb(led_id);
	return (OK);
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
TAP_ESC_RGBLED::send_led_rgb(int led_id)
{

	if (_enable) {

		_get_rgbled_color[led_id] = _set_rgbled_color[led_id];

	} else {
		_get_rgbled_color[led_id] = 0;
	}

	return (OK);
}

int
TAP_ESC_RGBLED::get(bool &on, bool &powersave, uint16_t &rgb, int led_id)
{
	powersave = OK; on = _enable;
	rgb = get_tap_esc_rgbled_color(led_id);
	return OK;
}

void
tap_esc_rgbled_usage()
{
	warnx("missing command: tap_esc_rgbled_usage try 'start', 'test', 'info', 'off', 'stop'");
}

int
tap_esc_rgbled_main(int argc, char *argv[])
{
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			break;

		case 'b':
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
			errx(1, "already started");
		}

		if (tap_esc_rgbled == nullptr) {
			tap_esc_rgbled = new TAP_ESC_RGBLED();

			if (tap_esc_rgbled == nullptr) {
				errx(1, "new failed");
			}

			if (OK != tap_esc_rgbled->init()) {
				delete tap_esc_rgbled;
				tap_esc_rgbled = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	/* need the driver past this point */
	if (tap_esc_rgbled == nullptr) {
		warnx("not started");
		tap_esc_rgbled_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		fd = open(TAPESC_RGBLED_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " TAPESC_RGBLED_DEVICE_PATH);
		}

		rgbled_pattern_t pattern = { {RGBLED_COLOR_RED, RGBLED_COLOR_GREEN, RGBLED_COLOR_BLUE, RGBLED_COLOR_WHITE, RGBLED_COLOR_OFF, RGBLED_COLOR_OFF},
			{500, 500, 500, 500, 1000, 0 }	// "0" indicates end of pattern
		};

		ret = ioctl(fd, RGBLED_SET_ID, (unsigned long)0);
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
		fd = open(TAPESC_RGBLED_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " TAPESC_RGBLED_DEVICE_PATH);
		}

		ret = ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
		close(fd);

		/* delete the rgbled object if stop was requested, in addition to turning off the LED. */
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

