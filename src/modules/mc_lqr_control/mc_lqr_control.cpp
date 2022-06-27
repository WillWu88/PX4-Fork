/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mc_lqr_control_main.cpp
 * Multicopter lqr controller.
 * Replacing mc_att_controller and mc_rate_controller
 *
 * System Theory Lab, Jun 2022
 *
 * @author Will Wu <willwu@wustl.edu>
 */

#include "mc_lqr_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

// needs confirming: can two registerCallback exist at the same time?
bool MulticopterLQRControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback() ||
        !_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}

int MulticopterLQRControl::task_spawn(int argc, char **argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterLQRControl *instance = new MulticopterLQRControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterLQRControl::custom_command(int argc, char **argv[])
{
	return print_usage("unknown command");
}

int MulticopterLQRControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This implements the multicopter lqr controller. It takes attitude and rate setpoints
and outputs actuator control messages.

The controller uses a quad specific LQR feedback gain for attitude and rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_lqr_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;

}

void MulticopterLQRControl::parameters_updated()
{
	_lqr_controller.setLQRGain(_lqr_gain);
}

float MulticopterLQRControl::throttle_curve(float throttle_stick_input)
{
	const float throttle_min = _landed ? 0.0f : _param_mpc_manthr_min.get();

	// throttle_stick_input is in range [0, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling to hover throttle
		return throttle_min + throttle_stick_input * (_param_mpc_thr_max.get() - throttle_min);

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		return math::gradual3(throttle_stick_input,
				      0.f, .5f, 1.f,
				      throttle_min, _param_mpc_thr_hover.get(), _param_mpc_thr_max.get());
	}
}

void MulticopterLQRControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(q).psi();

	/* reset yaw setpoint to current position if needed */
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f) > 0.05f
		   || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

		const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(x*x + y*y)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_x_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_y_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_x_input_filter.update(_manual_control_setpoint.x * _man_tilt_max);
	_man_y_input_filter.update(_manual_control_setpoint.y * _man_tilt_max);
	const float x = _man_x_input_filter.getState();
	const float y = _man_y_input_filter.getState();

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(math::constrain(_manual_control_setpoint.z, 0.f, 1.f));
	attitude_setpoint.timestamp = hrt_absolute_time();

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

	// update attitude controller setpoint immediately
	_lqr_controller.setAttitudeSetpoint(q_sp, attitutde_setpoint.yaw_sp_move_rate);
	_thrust_setpoint_body = Vector3f(attitude_setpoint.thrust_body);
	_last_attitude_setpoint = attitude_setpoint.timestamp;
}

void MulticopterLQRControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on attitude and rate changes */
	vehicle_attitude_s v_att;
	vehicle_angular_velocity_s angular_velocity;
}
/**
 * Multicopter lqr control app start / stop handling function
 */
extern "C" __EXPORT int mc_lqr_control_main(int argc, char *argv[])
{
	return MulticopterLQRControl::main(argc, argv);
}
