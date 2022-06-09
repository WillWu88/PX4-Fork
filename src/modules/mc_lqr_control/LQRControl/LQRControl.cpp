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

void LQR::adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
{

}


void LQRControl::setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				      const matrix::Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}
