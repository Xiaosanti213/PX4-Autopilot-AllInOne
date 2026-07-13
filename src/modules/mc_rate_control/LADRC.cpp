/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file LADRC.cpp
 *
 * Linear Active Disturbance Rejection Control implementation.
 * Uses discrete-time pole placement for numerically stable ESO.
 */

#include "LADRC.hpp"

using namespace matrix;

void LADRC::setParameters(const Vector3f &b0, const Vector3f &wo, const Vector3f &wc)
{
	_b0 = b0;
	_wo = wo;
	_wc = wc;
	_kp = wc;
}

void LADRC::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				const Vector<bool, 3> &saturation_negative)
{
	_saturation_positive = saturation_positive;
	_saturation_negative = saturation_negative;
}

Vector3f LADRC::update(const Vector3f &rate, const Vector3f &rate_sp, const float dt)
{
	const float dt_safe = math::max(dt, 0.0001f);

	// ================================================================
	// First-run initialization: seed z1 with current measurement
	// ================================================================
	// After reset(), z1=z2=z3=0. If the drone is already moving (e.g.,
	// wind gust or initial conditions), the ESO will see a huge initial
	// innovation and z3 will spike, potentially causing instability.
	// Solution: on the first update, set z1 = rate so the innovation is 0.
	// ================================================================
	if (!_initialized) {
		_z1 = rate;
		_z2.zero();
		_z3.zero();
		_initialized = true;
	}

	Vector3f torque;

	for (int axis = 0; axis < 3; axis++) {
		// ================================================================
		// Compute discrete-time ESO gains using exact pole placement
		// ================================================================
		// Observer poles at z = exp(-wo * dt) (triple pole)
		// l1 = 3*(1-p), l2 = 3*(1-p)^2/T, l3 = (1-p)^3/T^2
		// where p = exp(-wo * dt)
		// ================================================================
		const float w = math::max(_wo(axis), 1.0f);
		const float p = expf(-w * dt_safe);
		const float one_minus_p = 1.0f - p;

		const float l1 = 3.0f * one_minus_p;
		const float l2 = 3.0f * one_minus_p * one_minus_p / dt_safe;
		const float l3 = one_minus_p * one_minus_p * one_minus_p / (dt_safe * dt_safe);

		// ============================================================
		// Step 1: ESO Prediction
		// ============================================================
		const float u_prev = torque_prev(axis);

		const float z1_pred = _z1(axis) + dt_safe * _z2(axis);
		const float z2_pred = _z2(axis) + dt_safe * (_z3(axis) + _b0(axis) * u_prev);
		const float z3_pred = _z3(axis);

		// ============================================================
		// Step 2: ESO Correction (current estimator)
		// ============================================================
		const float innovation = rate(axis) - z1_pred;

		_z1(axis) = z1_pred + l1 * innovation;
		_z2(axis) = z2_pred + l2 * innovation;

		// Clamp z2 to reasonable angular acceleration range
		_z2(axis) = math::constrain(_z2(axis), -100.0f, 100.0f);

		_z3(axis) = z3_pred + l3 * innovation;

		// Clamp z3 to reasonable disturbance range
		// Max torque is ~2 N·m for roll/pitch, max b0=80 → max disturbance ~160 rad/s²
		// Use 200 as a safe upper bound
		_z3(axis) = math::constrain(_z3(axis), -200.0f, 200.0f);

		// ============================================================
		// Step 3: Control Law
		// ============================================================
		// PD control on estimated rate: u0 = kp * (rate_sp - z1)
		const float u0 = _kp(axis) * (rate_sp(axis) - _z1(axis));

		// Disturbance rejection: u = (u0 - z3) / b0
		float u = (u0 - _z3(axis)) / math::max(_b0(axis), 1.0f);

		// Anti-windup: if saturated, do not increase output in that direction
		if (_saturation_positive(axis)) {
			u = math::min(u, 0.0f);
		}

		if (_saturation_negative(axis)) {
			u = math::max(u, 0.0f);
		}

		// Clamp to normalized torque range
		torque(axis) = math::constrain(u, -1.0f, 1.0f);
	}

	// Store saturated control for next ESO iteration
	torque_prev = torque;

	return torque;
}

void LADRC::reset()
{
	_z1.zero();
	_z2.zero();
	_z3.zero();
	torque_prev.zero();
	_initialized = false;
}
