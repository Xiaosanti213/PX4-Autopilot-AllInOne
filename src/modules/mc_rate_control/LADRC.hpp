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
 * @file LADRC.hpp
 *
 * Linear Active Disturbance Rejection Control (LADRC) for angular rate control.
 *
 * Based on the ADRC framework by Prof. Han Jingqing (韩京清).
 * The core idea: treat all uncertainties (modeling errors, coupling, external
 * disturbances) as a "total disturbance" and estimate + cancel it in real-time
 * using an Extended State Observer (ESO).
 *
 * Architecture per axis:
 *   - 3rd-order ESO: estimates rate (z1), acceleration (z2), total disturbance (z3)
 *   - PD control law: u0 = kp * (rate_sp - z1)
 *   - Disturbance rejection: u = (u0 - z3) / b0
 *
 * Bandwidth parameterization (Gao Zhiqiang):
 *   - Observer poles all at -wo: beta1=3*wo, beta2=3*wo^2, beta3=wo^3
 *   - Controller pole at -wc: kp = wc
 *
 * @author PX4 Development Team
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class LADRC
{
public:
	LADRC() = default;
	~LADRC() = default;

	/**
	 * Set the LADRC parameters for all axes.
	 *
	 * @param b0   Control effectiveness gain (estimated input gain).
	 *             Typically 1/Inertia for rigid-body rate dynamics.
	 *             Roll/Pitch: ~80-200, Yaw: ~40-80 (PX4 normalized units).
	 * @param wo   Observer bandwidth [rad/s]. Higher = faster disturbance estimation,
	 *             but more noise-sensitive. Typical: 30-80 rad/s.
	 * @param wc   Controller bandwidth [rad/s]. Higher = faster tracking,
	 *             but may excite unmodeled dynamics. Typical: 20-50 rad/s.
	 */
	void setParameters(const matrix::Vector3f &b0, const matrix::Vector3f &wo, const matrix::Vector3f &wc);

	/**
	 * Set saturation status from control allocation feedback.
	 * When saturated, ESO update is clamped to prevent windup.
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	/**
	 * Run one LADRC control loop iteration.
	 *
	 * @param rate      Current angular rate measurement [rad/s]
	 * @param rate_sp   Desired angular rate setpoint [rad/s]
	 * @param dt        Time step [s]
	 * @return          Normalized torque command [-1, 1]
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp, const float dt);

	/**
	 * Reset all ESO states to zero (e.g., on disarm).
	 */
	void reset();

	/**
	 * Get the estimated total disturbance for diagnostics.
	 */
	const matrix::Vector3f &getDisturbance() const { return _z3; }

	/**
	 * Get the estimated rate for diagnostics.
	 */
	const matrix::Vector3f &getEstimatedRate() const { return _z1; }

private:
	// --- Parameters ---
	matrix::Vector3f _b0{80.f, 80.f, 40.f};   ///< control effectiveness
	matrix::Vector3f _wo{50.f, 50.f, 30.f};   ///< observer bandwidth [rad/s]
	matrix::Vector3f _wc{30.f, 30.f, 20.f};   ///< controller bandwidth [rad/s]

	// --- Controller gain ---
	matrix::Vector3f _kp;      ///< wc (proportional gain)

	// --- ESO states ---
	matrix::Vector3f _z1{};    ///< estimated angular rate
	matrix::Vector3f _z2{};    ///< estimated angular acceleration
	matrix::Vector3f _z3{};    ///< estimated total disturbance

	// --- Saturation flags ---
	matrix::Vector<bool, 3> _saturation_positive{};
	matrix::Vector<bool, 3> _saturation_negative{};

	// --- Previous control output (for ESO anti-windup) ---
	matrix::Vector3f torque_prev{};

	// --- Initialization flag ---
	bool _initialized{false};
};