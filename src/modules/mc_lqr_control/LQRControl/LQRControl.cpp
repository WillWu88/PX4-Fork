/**
 * @file LQRControl.cpp
 * LQR Attitude and Rate Controller
 *
 *
 * @author Will Wu(you@domain.com)
 * @date 2022-06-07
 *
 * @copyright Copyright (c) System Theory Lab, 2022
 *
 */
#include <LQRControl.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/defines.h>


matrix::Vector3f LQRControl::convertEigen(const Eigen::Vector3f &eigen_vector)
{
	matrix::Vector3f new_vec;
	for (int i = 0; i < eigen_vector.size(); i++){
		new_vec(i) = eigen_vector(i);
	}
	return new_vec;
}

Eigen::Vector3f LQRControl::convertPX4Vec(const matrix::Vector3f &px4_vector)
{
	Eigen::Vector3f new_vec;
	for (int i = 0; i < 3; i++){
		new_vec(i) << px4_vector(i);
	}
	return new_vec;
}

Eigen::Quaternionf LQRControl::convertQuatf(const matrix::Quatf &px4_quat)
{
	Eigen::Quaternionf new_vec(qd(0), qd(1), qd(2), qd(3));
	return new_vec;
}

void LQRControl::setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
{

	_attitude_setpoint = convertQuatf(qd);
	_attitude_setpoint.normalize();
	_yawspeed_setpoint = yawspeed_setpoint;
}

void LQRControl::adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
{
	Eigen::Quaternionf quat_shift = convertQuatf(q_delta);
	_attitude_setpoint *= quat_shift;
	_attitude_setpoint.normalize();
}

void LQRControl::setRateSetpoint(const matrix::Vector3f &new_rate_setpoint)
{
	_rate_setpoint = convertPX4Vec(new_rate_setpoint);
}

Eigen::Vector3f LQRControl::reduceQuat(const Eigen::Quaternionf &quat_cord)
{
	if (quat_cord.norm() == 1){
		Eigen::Vector3f new_vec(quat_cord(1), quat_cord(2), quat_cord(3));
		return new_vec;
	}
	else {
		Eigen::Quaternionf new_vec(quat_cord); //check copy constructor
		new_vec.normalize();
		Eigen::Vector3f return_vec(new_vec(1), new_vec(2), new_vec(3));
		return return_vec;
	}
}


Eigen::Matrix<float, 6, 1> LQRControl::constructState(const matrix::Vector3f &rate_state, const matrix::Quatf &q_state)
{
	Eigen::Matrix<float, 6, 1> state_vector;
	state_vector << reduceQuat(convertQuatf(q_state)-_attitude_setpoint), rate_state - _rate_setpoint;
	return state_vector;
}


void LQRControl::setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				      const matrix::Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

matrix::Vector3f LQRControl::update(const matrix::Quatf &q_state, const matrix::Vector3f &rate_state, const bool landed)
{
	Eigen::Vector3f torque = _lqr_gain_matrix * constructState(rate_state, q_state);
	return convertEigen(torque);
}
