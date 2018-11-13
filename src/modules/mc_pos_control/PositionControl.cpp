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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include "Utility/ControlMath.hpp"

using namespace matrix;

PositionControl::PositionControl(ModuleParams *parent) :
	ModuleParams(parent)
{}

void PositionControl::updateState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

bool PositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acc_x, setpoint.acc_y, setpoint.acc_z);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	bool mapping_succeeded = _interfaceMapping(); // bymark sp --> desired sp 用于前馈， 分别考虑位置控制，速度控制，以及都不控的情况

	// If full manual is required (thrust already generated), don't run position/velocity
	// controller and just return thrust.
	// bymark 在自稳飞行任务下，推力矢量sp已经生成了，就不需要执行位置/速度控制了，但在定高飞行任务下，推力矢量sp[2]还没生成
	_skip_controller = PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1])
			   && PX4_ISFINITE(setpoint.thrust[2]);

	return mapping_succeeded;
}

void PositionControl::generateThrustYawSetpoint(const float dt)
{
	if (_skip_controller) {
		// Already received a valid thrust set-point.
		// Limit the thrust vector.
		float thr_mag = _thr_sp.length();

		if (thr_mag > MPC_THR_MAX.get()) {
			_thr_sp = _thr_sp.normalized() * MPC_THR_MAX.get();

		} else if (thr_mag < MPC_MANTHR_MIN.get() && thr_mag > FLT_EPSILON) {
			_thr_sp = _thr_sp.normalized() * MPC_MANTHR_MIN.get();
		}

		// Just set the set-points equal to the current vehicle state.
		_pos_sp = _pos;
		_vel_sp = _vel;
		_acc_sp = _acc;

	} else {
		_positionController(); // bymark 由P控制+前馈给出vel_sp(2), 由P控制给出vel_sp(0:1)
		_velocityController(dt); // bymark 由PID控制+抗积分饱和给出thr_sp
	}
}

bool PositionControl::_interfaceMapping()
{
	// if noting is valid, then apply failsafe landing
	bool failsafe = false;

	// Respects FlightTask interface, where NAN-set-points are of no interest
	// and do not require control. A valide position and velocity setpoint will
	// be mapped to a desired position setpoint with a feed-forward term.
	// bymark 并不关心sp为NAN的量，因为不控它们。对于有效的pos_sp和vel_sp,将其映射为desired sp，好用于前馈
	for (int i = 0; i <= 2; i++) {	// bymark 三个通道分别考虑各自的前馈使用情况

		if (PX4_ISFINITE(_pos_sp(i))) { // bymark 需要位置控制环，进行位置控制
			// Position control is required

			if (!PX4_ISFINITE(_vel_sp(i))) { // bymark 当vel_sp为NAN时，不用速度前馈
				// Velocity is not used as feedforward term.
				_vel_sp(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control 
			// bymark 在控位置时，thr_sp应该由控制器给出
			_thr_sp(i) = 0.0f;

			// to run position control, we require valid position and velocity
			// bymark 在使用位置控制器时，位置和速度反馈量应该是有效的,否则进入失效保护
			if (!PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_vel_sp(i))) {	// bymark 不控位置，控速度

			// Velocity controller is active without position control.
			// Set the desired position set-point equal to current position
			// such that error is zero.
			// bymark 由于不控位置，所以让位置偏差为0，即pos_sp就是当前pos
			if (PX4_ISFINITE(_pos(i))) {
				_pos_sp(i) = _pos(i);

			} else {
				_pos_sp(i) = _pos(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control
			// bymark 在控速度时，thr_sp应该由控制器给出
			_thr_sp(i) = 0.0f;

			// to run velocity control, we require valid velocity
			// bymark 在使用速度控制器时，速度反馈量应该是有效的，否则进入失效保护
			if (!PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_thr_sp(i))) { // bymark 位置，速度都不控， thr_sp直接由杆量生成，所以将位置偏差和速度偏差都设为0

			// Thrust setpoint was generated from sticks directly.
			// Set all other set-points equal MC states.
			if (PX4_ISFINITE(_pos(i))) {
				_pos_sp(i) = _pos(i);

			} else {
				_pos_sp(i) = _pos(i) = 0.0f;
			}

			if (PX4_ISFINITE(_vel(i))) {
				_vel_sp(i) = _vel(i);

			} else {
				_vel_sp(i) = _vel(i) = 0.0f;
			}

			// Reset the Integral term.
			// bymark 积分项重置为0
			_thr_int(i) = 0.0f;
			// Don't require velocity derivative.
			// bymark 不使用速度微分
			_vel_dot(i) = 0.0f;

		} else {
			// nothing is valid. do failsafe
			// bymark 当pos_sp, vel_sp和thr_sp没有一个有效时，进入失效保护 
			failsafe = true;
		}
	}

	// ensure that vel_dot is finite, otherwise set to 0
	// bymark 确保vel_dot有效，否则置为0
	if (!PX4_ISFINITE(_vel_dot(0)) || !PX4_ISFINITE(_vel_dot(1))) {
		_vel_dot(0) = _vel_dot(1) = 0.0f;
	}

	if (!PX4_ISFINITE(_vel_dot(2))) {
		_vel_dot(2) = 0.0f;
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) { // bymark 确保yawspeed_sp有效，否则置为0
		// Set the yawspeed to 0 since not used.
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {	// bymark 确保yaw_sp有效，否则置为当前yaw
		// Set the yaw-sp equal the current yaw.
		// That is the best we can do and it also
		// agrees with FlightTask-interface definition.
		if (PX4_ISFINITE(_yaw)) {
			_yaw_sp = _yaw;

		} else {
			failsafe = true;
		}
	}

	// check failsafe
	if (failsafe) { // bymark 进入失效保护，设置thr_sp
		// point the thrust upwards
		_thr_sp(0) = _thr_sp(1) = 0.0f;
		// throttle down such that vehicle goes down with
		// 70% of throttle range between min and hover
		_thr_sp(2) = -(MPC_THR_MIN.get() + (MPC_THR_HOVER.get() - MPC_THR_MIN.get()) * 0.7f);
	}

	return !(failsafe);
}

void PositionControl::_positionController()
{
	// P-position controller + 前馈
	const Vector3f vel_sp_position = (_pos_sp - _pos).emult(Vector3f(MPC_XY_P.get(), MPC_XY_P.get(), MPC_Z_P.get()));
	_vel_sp = vel_sp_position + _vel_sp;

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	// bymark 对P控制器输出值进行限幅，不考虑前馈量，并且只是约束水平速度， 所以水平控制不用速度前馈？？
	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
				   Vector2f(_vel_sp - vel_sp_position), _constraints.speed_xy);
	_vel_sp(0) = vel_sp_xy(0);
	_vel_sp(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}

void PositionControl::_velocityController(const float &dt)
{

	// Generate desired thrust setpoint. 速度控制器给出是thr_sp
	// PID
	// u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)  Note: vel_err = vel_sp - vel
	// Umin <= u_des <= Umax
	//
	// Anti-Windup:
	// u_des = _thr_sp; r = _vel_sp; y = _vel
	// u_des >= Umax and r - y >= 0 ===> Saturation = true  达到上限，并且偏差为正
	// u_des >= Umax and r - y <= 0 ===> Saturation = false 达到上限，并且偏差为负
	// u_des <= Umin and r - y <= 0 ===> Saturation = true  达到下限，并且偏差为负
	// u_des <= Umin and r - y >= 0 ===> Saturation = false 达到下限，并且偏差为正
	//
	// 	Notes:
	// - PID implementation is in NED-frame  地面坐标系
	// - control output in D-direction has priority over NE-direction  垂直速度控制的优先级高于水平速度控制
	// - the equilibrium point for the PID is at hover-thrust
	// - the maximum tilt cannot exceed 90 degrees. This means that it is
	// 	 not possible to have a desired thrust direction pointing in the positive
	// 	 D-direction (= downward)
	// - the desired thrust in D-direction is limited by the thrust limits
	// - the desired thrust in NE-direction is limited by the thrust excess after
	// 	 consideration of the desired thrust in D-direction. In addition, the thrust in
	// 	 NE-direction is also limited by the maximum tilt.

	const Vector3f vel_err = _vel_sp - _vel;

	// Consider thrust in D-direction.
	// bymark D方向的PID运算
	float thrust_desired_D = MPC_Z_VEL_P.get() * vel_err(2) +  MPC_Z_VEL_D.get() * _vel_dot(2) + _thr_int(
					 2) - MPC_THR_HOVER.get();

	// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -MPC_THR_MIN.get();
	float uMin = -MPC_THR_MAX.get();

	// Apply Anti-Windup in D-direction.
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

	if (!stop_integral_D) {
		_thr_int(2) += vel_err(2) * MPC_Z_VEL_I.get() * dt;

		// limit thrust integral
		_thr_int(2) = math::min(fabsf(_thr_int(2)), MPC_THR_MAX.get()) * math::sign(_thr_int(2));
	}

	// Saturate thrust setpoint in D-direction.
	// bymark D方向的PID运算后进行限幅处理
	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

	if (fabsf(_thr_sp(0)) + fabsf(_thr_sp(1))  > FLT_EPSILON) { // bymark 当thr_sp(0:1)已经被提供后，例如定高模式， 只需使用最大倾斜角对其进行缩放
		// Thrust set-point in NE-direction is already provided. Only
		// scaling by the maximum tilt is required.
		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;

	} else { // bymark 用PID计算thr_sp(0:1)
		// PID-velocity controller for NE-direction.
		Vector2f thrust_desired_NE;
		thrust_desired_NE(0) = MPC_XY_VEL_P.get() * vel_err(0) + MPC_XY_VEL_D.get() * _vel_dot(0) + _thr_int(0);
		thrust_desired_NE(1) = MPC_XY_VEL_P.get() * vel_err(1) + MPC_XY_VEL_D.get() * _vel_dot(1) + _thr_int(1);

		// Get maximum allowed thrust in NE based on tilt and excess thrust.
		// bymark 根据最大倾斜角和最大推力来计算推力水平方分量的最大值thrust_max_NE，这是幅值
		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		float thrust_max_NE = sqrtf(MPC_THR_MAX.get() * MPC_THR_MAX.get() - _thr_sp(2) * _thr_sp(2));
		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

		// Saturate thrust in NE-direction.
		_thr_sp(0) = thrust_desired_NE(0);
		_thr_sp(1) = thrust_desired_NE(1);

		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) { // bymark PID输出值（模值的平方）> thrust_max_NE的平方
			float mag = thrust_desired_NE.length();
			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
		}

		// Get the direction of (r-y) in NE-direction.
		float direction_NE = Vector2f(vel_err) * Vector2f(_vel_sp);

		// Apply Anti-Windup in NE-direction.
		bool stop_integral_NE = (thrust_desired_NE * thrust_desired_NE >= thrust_max_NE * thrust_max_NE &&
					 direction_NE >= 0.0f);

		if (!stop_integral_NE) {
			_thr_int(0) += vel_err(0) * MPC_XY_VEL_I.get() * dt;
			_thr_int(1) += vel_err(1) * MPC_XY_VEL_I.get() * dt;

			// magnitude of thrust integral can never exceed maximum throttle in NE
			float integral_mag_NE = Vector2f(_thr_int).length();

			if (integral_mag_NE > 0.0f && integral_mag_NE > thrust_max_NE) {
				_thr_int(0) = _thr_int(0) / integral_mag_NE * thrust_max_NE;
				_thr_int(1) = _thr_int(1) / integral_mag_NE * thrust_max_NE;
			}
		}
	}
}

void PositionControl::updateConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.

	if (!PX4_ISFINITE(constraints.tilt)
	    || !(constraints.tilt < math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get()))) {
		_constraints.tilt = math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get());
	}

	if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < MPC_Z_VEL_MAX_UP.get())) {
		_constraints.speed_up = MPC_Z_VEL_MAX_UP.get();
	}

	if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < MPC_Z_VEL_MAX_DN.get())) {
		_constraints.speed_down = MPC_Z_VEL_MAX_DN.get();
	}

	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < MPC_XY_VEL_MAX.get())) {
		_constraints.speed_xy = MPC_XY_VEL_MAX.get();
	}
}

void PositionControl::updateParams()
{
	ModuleParams::updateParams();

	// Tilt needs to be in radians
	MPC_TILTMAX_AIR_rad.set(math::radians(MPC_TILTMAX_AIR_rad.get()));
	MPC_MAN_TILT_MAX_rad.set(math::radians(MPC_MAN_TILT_MAX_rad.get()));
}
