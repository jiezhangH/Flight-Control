/*
 * fault_tolerant_control.cpp
 *
 *  Created on: Mar 12, 2017
 *      Author: Haohua Chang
 */
#include "fault_tolerant_control.h"
#include <drivers/tap_esc/drv_tap_esc.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/esc_status.h>
#include <float.h>
#include <errno.h>
#include <px4_defines.h>
#include <lib/mathlib/mathlib.h>

FaultTolerantControl::FaultTolerantControl(const char *geomname) :
	_filter_delta_pwm(100.0f, 50.0f),
	_params_sub(-1)
{
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	if (get_parameters_handle() == OK) {
		get_parameters_update();
	}

}

FaultTolerantControl::~FaultTolerantControl()
{

}

int FaultTolerantControl::get_parameters_handle()
{
	_params_handles.motor_failure_gain		=	param_find("FTC_GAIN");
	_params_handles.propeller_reverse_coefficient	=	param_find("PROPELL_REV_COEF");

	return OK;
}

int FaultTolerantControl::get_parameters_update()
{
	float p;

	param_get(_params_handles.motor_failure_gain,			&p);	_params.motor_failure_gain 	= p;
	param_get(_params_handles.propeller_reverse_coefficient, &p);	_params.propeller_reverse_coefficient 	= p;

	return OK;
}

void FaultTolerantControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		get_parameters_update();
	}
}

/**
 * the proportion according to S-Curve calculate compensatory gain
 * @param x:pwm error(pwm max - pwm min) x rang is (-1,1)
 * return: auto gain
 */
float FaultTolerantControl::s_curve_gain(float x)
{
	float K = 2.0f;
	float a = 2.0f;
	float b = 0.6f;
	return K * (1 / (1.0f + b * expf(-a * x * x)) - 0.5f);
}

/**
 * recalculate pwm output,when the vehicle enter fault tolerant control mode
 * vehicle model hexa_x:
 *	   3     2
 *	    \   /
 *	     \ /
 *	4---- X ----1
 *	     / \
 *	    /   \
 *	   5     0
 * motor0,2,4:CCW
 * motor1,3,5:CW
 * @param failure_pwm: input failure motor pwm value(e.g.1000~2000)
 * @param diagonal_pwm: input pwm value the motor with the failure motor is diagonal
 * @param delta:output delta pwm only for debug
 * @param pwm_output:output pwm(e.g.1000~2000)
 */
uint16_t FaultTolerantControl::recalculate_pwm_outputs(uint16_t failure_pwm, uint16_t diagonal_pwm, float &delta)
{
	// the motor pwm merge with failure motor pwm and filter
	float delta_pwm = diagonal_pwm - failure_pwm;
	delta_pwm = math::constrain(delta_pwm, (float)(RPMMIN - RPMMAX), (float)(RPMMAX - RPMMIN));
	//delta_pwm = _filter_delta_pwm.apply(delta_pwm);

	// compensate for the motor failure case roll, pitch and yaw output will not balance
	float nonlinear_gain = _params.motor_failure_gain * s_curve_gain(delta_pwm / (RPMMAX - RPMMIN));
	delta_pwm = nonlinear_gain * delta_pwm;

	delta = delta_pwm;

	// switch motor direction of rotation is forward-rotating or reverse
	if (delta_pwm >= 0.0f) {
		diagonal_pwm = RPMMIN + delta_pwm;

	} else {
#ifdef ESC_SUPPORT_REVERSE
		diagonal_pwm = RPMMIN - delta_pwm * _params.propeller_reverse_coefficient;
		diagonal_pwm |= RUN_REVERSE_MASK;
#else
		diagonal_pwm = RPMSTOPPED;
#endif
	}

	return diagonal_pwm;
}
