/**
 * @file LQRControl.hpp
 *
 * LQR Attitude and Rate Controller
 *
 * @author Will Wu	<willwu@wustl.edu>
 *
 * @ref Matthias Grob <AttitudeControl.hpp> <AttitudeControl.cpp>
 * @ref <RateControl.cpp> <RateControl.hpp>
 * @copyright Copyright (c) System Theory Lab, 2022
 */
#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <eigen/Dense>


#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>

class LQRControl
{
public:
	LQRControl() = default;
	~LQRControl() = default;

	/**
	 * Set new lqr gain matrix (6x3)
	 * @param new_k new gain matrix
	 */
	void setLQRGain(const Eigen::Matrix<float, 3 , 6> &new_k) {_lqr_gain_matrix = new_k;}

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

 	/**
  	* Update Rate Setpoint
  	* @param new_rate_setpoint
  	*/
	void setRateSetpoint(const matrix::Vector3f &new_rate_setpoint) {_rate_setpoint = new_rate_setpoint};

	/**
	 * Construct error state for gain multiplication
	 *
	 * @param rate_state current angular rate from estimator
	 * @param q_state current rotation from estimator
	 * @return error vector for gain multiplication
	 */
	Eigen::Matrix<float, 6, 1> construct_state(const matrix::Vector3f &rate_state, const matrix::Quatf &q_state);

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	/**
	 * Run the controller once
	 *
	 * @param q_state current rotation from the estimator
	 * @param rate_state current rate from the estimator
	 * @param landed vehicle landing status
	 * @warning need to implement landing logic
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Quatf &q_state, const matrix::Vector3f &rate_state,
				const bool landed)

private:
	const int _num_of_output = 3;
	const int _num_of_states = 6;
	Eigen::Vector4d _attitude_setpoint;
	Eigen::MatrixXf _lqr_gain_matrix(_num_of_output, _num_of_states);
	matrix::Vector3f _rate_setpoint;
	float _yaw_speed_setpoint{0.f};

	//Optional Params
	matrix::Vector3f _rate_limit;

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;

};
