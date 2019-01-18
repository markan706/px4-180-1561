/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightManualStabilized.cpp
 */

#include "FlightTaskManualStabilized.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualStabilized::activate()
{
	bool ret = FlightTaskManual::activate();
	_thrust_setpoint = matrix::Vector3f(0.0f, 0.0f, -_throttle_hover.get());
	_yaw_setpoint = _yaw;
	_yawspeed_setpoint = 0.0f;
	_constraints.tilt = math::radians(_tilt_max_man.get());
	return ret;
}

bool FlightTaskManualStabilized::updateInitialize()
{
	bool ret = FlightTaskManual::updateInitialize();
	// need a valid yaw-state
	return ret && PX4_ISFINITE(_yaw);
}

void FlightTaskManualStabilized::_scaleSticks()
{
	/* Scale sticks to yaw and thrust using
	 * linear scale for yaw and piecewise linear map for thrust. */
	_yawspeed_setpoint = _sticks_expo(3) * math::radians(_yaw_rate_scaling.get()); // bymark 偏航角速度sp = 调整后的偏航杆量 × MPC_MAN_Y_MAX(200deg/s)
	_throttle = _throttleCurve(); // bymark 油门映射，获取油门指令大小 [0，1]
}

void FlightTaskManualStabilized::_updateHeadingSetpoints()
{
	/* Yaw-lock depends on stick input. If not locked,
	 * yaw_sp is set to NAN.
	 * TODO: add yawspeed to get threshold.*/
	if (fabsf(_yawspeed_setpoint) > FLT_EPSILON) { // bymark 有yaw杆量产生了偏航角速度sp
		// no fixed heading when rotating around yaw by stick
		_yaw_setpoint = NAN;

	} else {
		// hold the current heading when no more rotation commanded
		if (!PX4_ISFINITE(_yaw_setpoint)) { // bymark 当对yaw不再打杆时，放弃对yaw_speed的控制
			_yaw_setpoint = _yaw;

		} else {
			// check reset counter and update yaw setpoint if necessary
			if (_sub_attitude->get().quat_reset_counter != _heading_reset_counter) {
				_yaw_setpoint += matrix::Eulerf(matrix::Quatf(_sub_attitude->get().delta_q_reset)).psi();
				_heading_reset_counter = _sub_attitude->get().quat_reset_counter;
			}
		}
	}

	// check if an external yaw handler is active and if yes, let it compute the yaw setpoints
	// bymark 外部yaw handler 与weathervane (风标控制器)有关
	if (_ext_yaw_handler != nullptr && _ext_yaw_handler->is_active()) {
		_yaw_setpoint = NAN;
		_yawspeed_setpoint += _ext_yaw_handler->get_weathervane_yawrate();
	}
}

void FlightTaskManualStabilized::_updateThrustSetpoints()
{
	/* Rotate setpoint into local frame. */
	Vector2f sp(&_sticks(0));
	_rotateIntoHeadingFrame(sp);

	// bymark xy杆量直接映射为roll_sp, pitch_sp的缩放因子(将滚转和俯仰合成倾斜角)

	/* Ensure that maximum tilt is in [0.001, Pi] */
	float tilt_max = math::constrain(_constraints.tilt, 0.001f, M_PI_F);

	const float x = sp(0) * tilt_max;
	const float y = sp(1) * tilt_max;

	/* The norm of the xy stick input provides the pointing
	 * direction of the horizontal desired thrust setpoint. The magnitude of the
	 * xy stick inputs represents the desired tilt. Both tilt and magnitude can
	 * be captured through Axis-Angle.
	 */
	/* The Axis-Angle is the perpendicular vector to xy-stick input */
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > tilt_max) { // limit to the configured maximum tilt angle
		v *= tilt_max / v_norm;
	}

	/* The final thrust setpoint is found by rotating the scaled unit vector pointing
	 * upward by the Axis-Angle.
	 * Make sure that the attitude can be controlled even at 0 throttle.
	 */
	Quatf q_sp = AxisAnglef(v(0), v(1), 0.0f);
	// bymark 用单位四元素q_sp对机体坐标系下的期望推力矢量（0，0，-_throttle）进行旋转
	_thrust_setpoint = q_sp.conjugate(Vector3f(0.0f, 0.0f, -1.0f)) * math::max(_throttle, 0.0001f);
}

void FlightTaskManualStabilized::_rotateIntoHeadingFrame(Vector2f &v)
{
	float yaw_rotate = PX4_ISFINITE(_yaw_setpoint) ? _yaw_setpoint : _yaw;
	Vector3f v_r = Vector3f(Dcmf(Eulerf(0.0f, 0.0f, yaw_rotate)) * Vector3f(v(0), v(1), 0.0f));
	v(0) = v_r(0);
	v(1) = v_r(1);
}

void FlightTaskManualStabilized::_updateSetpoints()
{
	_updateHeadingSetpoints(); // bymark 在考虑yawspeed_sp的情况下更新yaw_sp
	_updateThrustSetpoints();  // bymark 结合xy杆量（期望推力矢量的水平方向），将油门指令大小thr转换成推力矢量thr_sp
}

float FlightTaskManualStabilized::_throttleCurve()
{
	// Scale stick z from [-1,1] to [min thrust, max thrust]
	float throttle = -((_sticks(2) - 1.0f) * 0.5f);  // bymark 把油门杆量从[-1，1]缩放到[0，1]

	switch (_throttle_curve.get()) {
	case 1: // no rescaling
		return throttle;

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		if (throttle < 0.5f) {
			return (_throttle_hover.get() - _throttle_min_stabilized.get()) / 0.5f * throttle + _throttle_min_stabilized.get();

		} else {
			return (_throttle_max.get() - _throttle_hover.get()) / 0.5f * (throttle - 1.0f) + _throttle_max.get();
		}
	}

}

bool FlightTaskManualStabilized::update()
{
	_scaleSticks(); // bymark  处理油门和偏航杆量，更新油门thr和yawspeed_sp（自稳飞行任务）
	_updateSetpoints();// bymark 更新yaw_sp和thr_sp（自稳飞行任务）

	return true;
}
