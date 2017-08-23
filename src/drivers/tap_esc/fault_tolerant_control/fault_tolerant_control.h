/*
 * fault_tolerant_control.h
 *
 *  Created on: Mar 12, 2017
 *      Author: Haohua Chang
 */

#ifndef FAULT_TOLERANT_CONTROL_H_
#define FAULT_TOLERANT_CONTROL_H_

#include <stdint.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#define ESC_SUPPORT_REVERSE

class FaultTolerantControl
{
public:

	/**
	 * Constructor
	 */
	FaultTolerantControl(const char *geomname);

	/**
	 * Destructor
	 */
	~FaultTolerantControl();

	/**
	 * the proportion according to S-Curve calculate compensatory gain
	 */
	float s_curve_gain(float x);

	/**
	 * Get parameter handle.
	 */
	int		get_parameters_handle();

	/**
	 * Update our local parameter cache.
	 */
	int		get_parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void 	parameter_update_poll();

	/**
	 * recalculate pwm output,when the vehicle enter fault tolerant control
	 */
	uint16_t recalculate_pwm_outputs(uint16_t failure_pwm, uint16_t diagonal_pwm, float &delta);
private:

	struct {
		param_t motor_failure_gain;
		param_t propeller_reverse_coefficient;
	}	_params_handles;		/**< handles for interesting parameters */

	struct {
		float motor_failure_gain;
		float propeller_reverse_coefficient;
	}	_params;

	math::LowPassFilter2p
	_filter_delta_pwm;	/**< filters for the delta pwm(e.g. pwm[1] - pwm[0]),to smooth error pwm will decision to motor reverse */
	int _params_sub;			/**< parameter updates subscription */

};


#endif /* FIVE_PROPELLERS_HANDLING_H_ */
