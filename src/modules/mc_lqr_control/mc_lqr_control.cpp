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
 * Replacing mc_control_torqueler and mc_rate_controller
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
MulticopterLQRControl::MulticopterLQRControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
}

MulticopterLQRControl::~MulticopterLQRControl()
{
	perf_free(_loop_perf);
}

bool MulticopterLQRControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback() ||
        !_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}

int MulticopterLQRControl::task_spawn(int argc, char *argv[])
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

int MulticopterLQRControl::custom_command(int argc, char *argv[])
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

void MulticopterLQRControl::publishTorqueSetpoint(const matrix::Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s vehicle_torque_setpoint{};
	vehicle_torque_setpoint.timestamp = hrt_absolute_time();
	vehicle_torque_setpoint.timestamp_sample = timestamp_sample;
	vehicle_torque_setpoint.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	vehicle_torque_setpoint.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	vehicle_torque_setpoint.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
}

void MulticopterLQRControl::publishThrustSetpoint(const float thrust_setpoint, const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
	vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
	vehicle_thrust_setpoint.timestamp_sample = timestamp_sample;
	vehicle_thrust_setpoint.xyz[0] = 0.0f;
	vehicle_thrust_setpoint.xyz[1] = 0.0f;
	vehicle_thrust_setpoint.xyz[2] = PX4_ISFINITE(thrust_setpoint) ? -thrust_setpoint : 0.0f; // Z is Down

	_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);
}

void MulticopterLQRControl::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
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

	if (_vehicle_attitude_sub.update(&v_att) || _vehicle_angular_velocity_sub.update(&angular_velocity)) {
		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;
		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		// run at 200 Hz
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.0002f, 0.02f);
		// compare updated time, take the most recent one
		_last_run = (angular_velocity.timestamp_sample > v_att.timestamp_sample) ? angular_velocity.timestamp_sample : v_att.timestamp_sample;

		const matrix::Quatf q{v_att.q};
		const matrix::Vector3f angular_accel{v_angular_acceleration.xyz};
		const matrix::Vector3f rates{angular_velocity.xyz};

		// Check for new attitude setpoint, update rates setpoint
		if (_vehicle_attitude_setpoint_sub.updated()) {
			vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

			if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
			    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

				_lqr_controller.setAttitudeSetpoint(matrix::Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
				_thrust_setpoint_body = matrix::Vector3f(vehicle_attitude_setpoint.thrust_body);
				_thrust_setpoint = _thrust_setpoint_body(2);
				_rates_setpoint = _default_rates;
				_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				_lqr_controller.setRateSetpoint(_rates_setpoint);
			}
		}

		// Check for a heading reset
		if (_quat_reset_counter != v_att.quat_reset_counter) {
			const matrix::Quatf delta_q_reset(v_att.delta_q_reset);

			// for stabilized attitude generation only extract the heading change from the delta quaternion
			_man_yaw_sp = matrix::wrap_pi(_man_yaw_sp + matrix::Eulerf(delta_q_reset).psi());

			if (v_att.timestamp > _last_attitude_setpoint) {
				// adapt existing attitude setpoint unless it was generated after the current attitude estimate
				_lqr_controller.adaptAttitudeSetpoint(delta_q_reset);
			}

			_quat_reset_counter = v_att.quat_reset_counter;
		}

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_vtol = vehicle_status.is_vtol;
			}
		}

		bool run_lqr = _lqr_test || _vehicle_control_mode.flag_control_lqr_enabled;

		if (run_lqr) {
			const matrix::Vector3f control_torque = _lqr_controller.update(q, rates, _landed);

			// rate controller status?
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(control_torque(0)) ? control_torque(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(control_torque(1)) ? control_torque(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(control_torque(2)) ? control_torque(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_setpoint) ? _thrust_setpoint : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			publishTorqueSetpoint(control_torque, angular_velocity.timestamp_sample);
			publishThrustSetpoint(_thrust_setpoint, angular_velocity.timestamp_sample);

			// battery control scalling

			actuators.timestamp = hrt_absolute_time();
			_actuator_controls_0_pub.publish(actuators);

			updateActuatorControlsStatus(actuators, dt);
		}
	}
	perf_end(_loop_perf);
}
/**
 * Multicopter lqr control app start / stop handling function
 */
extern "C" __EXPORT int mc_lqr_control_main(int argc, char *argv[])
{
	return MulticopterLQRControl::main(argc, argv);
}
