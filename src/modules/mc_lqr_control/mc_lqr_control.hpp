/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

/* Modified MC_Attitude_Rate_CTRL using lqr
* Author: Will Wu
* System Theory Lab
* Washington University in St. Louis, May 2022
*/

#pragma once

#include <lib/mixer/MixerBase/Mixer.hpp> // Airmode
#include <eigen/Dense>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

#include <modules/mc_lqr_control/LQRControl/LQRControl.hpp>

using namespace time_literals;

class MulticopterLQRControl: public ModuleBase<MulticopterLQRControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterLQRControl(bool vtol = false);
	~MulticopterLQRControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize vectors/matrices
	 */
	void parameters_updated() {}

	// throttle curve adjustment, taken from mc_att_control
	float throttle_curve(float throttle_stick_input);

	// generate attitude setpoint based on position
	void generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp);

	// publish control signals
	void updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt);

	void publishTorqueSetpoint(const matrix::Vector3f &torque_sp, const hrt_abstime &timestamp_sample);

	void publishThrustSetpoint(const float thrust_setpoint, const hrt_abstime &timestamp_sample);

    // Messaging specifics

	// log and performance counters
	hrt_abstime _last_run{0};
	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	// sampling interval
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// call back work items, status dependent
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// subscribed topics
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};   /**< vehicle attitude setpoint subscription */
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};             /**< vehicle control mode subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};                         /**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};           /**< vehicle land detected subscription */

	// publication topics
	uORB::Publication<actuator_controls_s>		_actuator_controls_0_pub;
	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_0_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};


	// lqr controller
	LQRControl _lqr_controller;

	// attitude setpoint conversion parameters
	float _man_yaw_sp{0.f};                 /**< current yaw setpoint in manual mode */
	float _man_tilt_max;                    /**< maximum tilt allowed for manual flight [rad] */
	AlphaFilter<float> _man_x_input_filter;
	AlphaFilter<float> _man_y_input_filter;
	matrix::Vector3f _thrust_setpoint_body; /**< body frame 3D thrust vector */
	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};
	// heading reset counter
	uint8_t _quat_reset_counter{0};

	// manual control switch and flight mode switch
	manual_control_setpoint_s       _manual_control_setpoint {};    /**< manual control setpoint */
	vehicle_control_mode_s          _vehicle_control_mode {};       /**< vehicle control mode */

	// rate control parameters
	bool _actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */
	bool _landed{true};
	bool _maybe_landed{true};

	// rate control integrator anti windup parameters
	float _energy_integration_time{0.0f};
	float _control_energy[4] {};


	// LQR Controller Gain
	Eigen::MatrixXf _lqr_gain; _lqr_gain << 0,0,0,0,0,0
								   0,0,0,0,0,0
								   0,0,0,0,0,0;

	//attitude rate setpoint, always set to 0
	matrix::Vector3f _rates_setpoint(0.0f, 0.0f, 0.0f);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>)         _param_mc_airmode,

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,

		/* Stabilized mode params */
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,    /**< maximum tilt allowed for manual flight */
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,       /**< scaling factor from stick to yaw rate */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>)   _param_mpc_manthr_min,      /**< minimum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,         /**< maximum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,       /**< throttle at stationary hover */
		(ParamInt<px4::params::MPC_THR_CURVE>)      _param_mpc_thr_curve        /**< throttle curve behavior */
	)

};
