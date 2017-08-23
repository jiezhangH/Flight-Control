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
 * @file five_propellers_handling_params.c
 * Parameters for multicopter breakdown(five propellers mode)attitude controller
 *
 * @author Haohua Chang
 */

/**
 * motor failure P gain compensate for fault tolerant control
 *
 * fault tolerant control handling when the motor(e.g motor0) have failure,error_pwm = (motor1 pwm - motor0 pwm)*(proportional gain compensate)
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_GAIN, 0.6f);

/**
 * the develop force effects when corotate propeller is equivalent to
 * the develop force effects when reverse propeller multiply by compensate coefficient.
 *
 * corotate propeller force = (reverse propellers force)*(proportional reverse compensatory coefficient)
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(PROPELL_REV_COEF, 1.2f);

/**
 * fault tolerant control mode roll rate P gain when motor have failure
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_ROLLRATE_P, 0.130f);

/**
 * fault tolerant control mode roll rate I gain when motor have failure
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_ROLLRATE_I, 0.06f);

/**
 * fault tolerant control mode roll rate D gain when motor have failure
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_ROLLRATE_D, 0.0f);

/**
 * fault tolerant control mode pitch rate P gain when motor have failure
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_PITCHRATE_P, 0.130f);

/**
 * fault tolerant control mode pitch rate I gain when motor have failure
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_PITCHRATE_I, 0.06f);

/**
 * fault tolerant control mode pitch rate D gain when motor have failure
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_PITCHRATE_D, 0.0f);

/**
 * fault tolerant control mode yaw rate P gain when one motor have failure
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_YAWRATE_P, 0.165f);

/**
 * fault tolerant control mode yaw rate I gain when one motor have failure
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_YAWRATE_I, 0.1f);

/**
 * fault tolerant control mode yaw rate D gain when one motor have failure
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Fault Tolerant Control
 */
PARAM_DEFINE_FLOAT(FTC_YAWRATE_D, 0.0f);
