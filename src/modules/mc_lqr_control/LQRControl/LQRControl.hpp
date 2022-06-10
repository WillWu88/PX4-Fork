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
	 * convert eigen lib vectors in px4 vectors
	 * @param eigen_vector vector to convert
	 * @return px4 3d vector
	 * */
	matrix::Vector3f convertEigen(const Eigen::Vector3f &eigen_vector);

	/**
	 * convert px4 vectors into eigen vectors
	 * @param eigen_vector vector to convert
	 * @return px4 3d vector
	 * */
	Eigen::Vector3f convertPX4Vec(const matrix::Vector3f &px4_vector);

	/**
	 * convert px4 quaternion into eigen quaternions
	 * @param px4_quat
	 * @return eigen quaternion
	 * */
	Eigen::Quaternionf convertQuatf(const matrix::Quatf &px4_quat);

	/**
	 * Set new lqr gain matrix (3x6)
	 * @param new_k new gain matrix
	 */
	void setLQRGain(const Eigen::Matrix<float, 3 , 6> &new_k) {_lqr_gain_matrix = new_k;}

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint);

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta);

 	/**
  	* Update Rate Setpoint
  	* @param new_rate_setpoint
  	*/
	void setRateSetpoint(const matrix::Vector3f &new_rate_setpoint);


	/**
	 * Reduce quaternion by eliminating the scalar
	 * @param quat_cord ref to the quaternion to change
	 * @return a 3x1 vector containing the i, j, k value
	 */
	Eigen::Vector3f reduceQuat(const Eigen::Quaternionf &quat_cord);

	/**
	 * Construct error state for gain multiplication
	 *
	 * @param rate_state current angular rate from estimator
	 * @param q_state current rotation from estimator
	 * @return error vector for gain multiplication
	 */
	Eigen::Matrix<float, 6, 1> constructState(const matrix::Vector3f &rate_state,
												   const matrix::Quatf &q_state);

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
				const bool landed);

private:
	const int _num_of_output = 3;
	const int _num_of_states = 6;
	Eigen::Quaternionf _attitude_setpoint; //storing in the order of x, y, z, w
	Eigen::MatrixXf _lqr_gain_matrix(_num_of_output, _num_of_states);
	Eigen::Vector3f _rate_setpoint;
	float _yawspeed_setpoint{0.f};


	//Optional Params
	matrix::Vector3f _rate_limit;

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;

};
