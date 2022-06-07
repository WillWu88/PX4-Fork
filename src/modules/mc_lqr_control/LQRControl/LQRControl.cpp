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

using namespace matrix;


void LQRControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				      const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

matrix::Vector3f update(const matrix::Quatf &q_state, const matrix::Vector3f &rate_state,
			const bool landed)
{
	// reduce quaternion states, since q0 is constrained by unit quaternion
	q_state.normalize();
	const Vector3f reduced_q_state;
	for (int i = 0; i < 3; i++) {
		reduced_q_state(i) = q_state(i+1);
	}

	const Vector3f torque;

	return torque;


}
