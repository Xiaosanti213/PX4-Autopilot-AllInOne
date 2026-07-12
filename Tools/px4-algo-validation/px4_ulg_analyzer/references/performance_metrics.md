# Flight Performance Metrics Reference

This document defines the performance metrics computed by the flight analyzer,
including formulas, interpretation guidelines, and quality thresholds.

## 1. Rate Tracking Performance

**What it measures**: How well the aircraft tracks commanded angular rate setpoints.

**Data sources**:
- Setpoint: `vehicle_rates_setpoint` (roll, pitch, yaw in rad/s)
- Actual: `vehicle_attitude` (rollspeed, pitchspeed, yawspeed in rad/s)

**Formulas**:
- Error = Setpoint - Actual (per axis, per sample)
- RMS Error = sqrt(mean(error^2))
- Mean Absolute Error = mean(|error|)
- Max Error = max(|error|)

**Quality thresholds** (RMS error in deg/s):

| Rating | Roll/Pitch | Yaw |
|--------|-----------|-----|
| Excellent | < 5 | < 3 |
| Good | 5-10 | 3-8 |
| Acceptable | 10-15 | 8-15 |
| Poor | 15-30 | 15-30 |
| Critical | > 30 | > 30 |

**Interpretation**: High rate tracking error indicates either aggressive maneuvers
exceeding controller bandwidth, insufficient gains, or external disturbances.

---

## 2. Attitude Tracking Performance

**What it measures**: How well the aircraft tracks commanded attitude (roll/pitch/yaw angles).

**Data sources**:
- Setpoint: `vehicle_attitude_setpoint` (roll_body, pitch_body, yaw_body in radians)
- Actual: `vehicle_attitude` (quaternion converted to Euler angles)

**Formulas**:
- Error = Setpoint - Actual (per axis, per sample, in degrees)
- RMS Error, Mean Absolute Error, Max Error (same as rate tracking)

**Quality thresholds** (RMS error in degrees):

| Rating | Roll/Pitch | Yaw |
|--------|-----------|-----|
| Excellent | < 2 | < 3 |
| Good | 2-5 | 3-8 |
| Acceptable | 5-10 | 8-15 |
| Poor | > 10 | > 15 |

**Interpretation**: Attitude tracking error reflects the outer-loop controller
performance. Consistent offset suggests trim or bias issues.

---

## 3. Position Tracking Performance

**What it measures**: How well the aircraft tracks commanded position setpoints.

**Data sources**:
- Setpoint: `vehicle_local_position_setpoint` (x, y, z in NED frame)
- Actual: `vehicle_local_position` (x, y, z in NED frame)

**Formulas**:
- Per-axis error = Setpoint - Actual
- 3D Error = sqrt(dx^2 + dy^2 + dz^2)
- RMS 3D Error = sqrt(mean(3D_error^2))

**Quality thresholds** (3D RMS error in meters):

| Rating | Hover/Static | Dynamic Flight |
|--------|-------------|----------------|
| Excellent | < 0.2 | < 0.5 |
| Good | 0.2-0.5 | 0.5-1.0 |
| Acceptable | 0.5-1.0 | 1.0-2.0 |
| Poor | > 1.0 | > 2.0 |

**Interpretation**: Position tracking error depends on GPS quality, wind conditions,
and controller tuning. Large errors during hover suggest position controller issues.

---

## 4. Vibration Analysis

**What it measures**: Mechanical vibration levels from raw IMU data.

**Data sources**:
- `sensor_gyro` (x_rad_s, y_rad_s, z_rad_s)
- `sensor_accel` (x_m_s2, y_m_s2, z_m_s2)

**Formulas**:
- Peak-to-Peak = max(values) - min(values)
- RMS = sqrt(mean(values^2))
- Std = standard deviation

**Quality thresholds** (peak-to-peak):

| Rating | Accel (m/s^2) | Gyro (rad/s) |
|--------|--------------|-------------|
| Good | < 15 | < 0.2 |
| Acceptable | 15-30 | 0.2-0.5 |
| High | 30-50 | 0.5-1.0 |
| Critical | > 50 | > 1.0 |

**Interpretation**: High vibration causes:
- Increased EKF innovation noise
- Accelerometer clipping (data loss)
- Poor attitude estimation
- Mechanical wear

**Mitigation**: Better IMU isolation (foam mount), balance propellers, check motor bearings.

---

## 5. Estimator (EKF) Performance

**What it measures**: Health and accuracy of the Extended Kalman Filter.

**Data sources**:
- `estimator_status` (test ratios)
- `estimator_innovations` (innovation values)

**Test ratios** (key indicator):
- `pos_test_ratio`: Position GPS vs EKF consistency
- `vel_test_ratio`: Velocity GPS vs EKF consistency
- `hgt_test_ratio`: Height barometer vs EKF consistency
- `mag_test_ratio`: Magnetometer vs EKF consistency

**Quality thresholds** (max test ratio):

| Rating | Test Ratio Range |
|--------|-----------------|
| Good | < 0.5 |
| Marginal | 0.5-1.0 |
| Poor | > 1.0 (sensor being rejected) |

**Interpretation**: Test ratio > 1.0 means the EKF is rejecting that sensor.
Consistently high ratios indicate sensor failure, magnetic interference, or
GPS multipath.

---

## 6. Battery & Power Analysis

**What it measures**: Battery performance and power consumption.

**Data sources**: `battery_status`

**Key metrics**:
- Voltage drop: V_start - V_end (large drop = high internal resistance or high load)
- Energy consumed: consumed_mah (total energy used during flight)
- Average power: mean(V * I) in Watts
- Peak current: max(current_a)

**Quality indicators**:
- Voltage drop > 2V under load: battery health concern
- Remaining < 20% at end: flight was close to low-battery threshold
- Current spikes: aggressive maneuvers or motor inefficiency

---

## 7. Actuator Saturation Analysis

**What it measures**: How often motor commands reach their limits.

**Data sources**:
- `actuator_motors` (control[0..7], normalized -1 to 1)
- `actuator_outputs` (output[0..15], PWM 1000-2000)

**Formulas**:
- Saturation = |control| >= 0.95 (normalized) or PWM >= 1950
- Saturation% = (saturated_samples / total_samples) * 100

**Quality thresholds** (per motor):

| Rating | Saturation % |
|--------|-------------|
| Good | < 5% |
| Acceptable | 5-10% |
| Concerning | 10-20% |
| Critical | > 20% |

**Interpretation**: Motor saturation means:
- Aircraft cannot achieve commanded torque
- Control authority is limited
- Risk of instability in aggressive maneuvers
- May need to reduce gains or increase motor size

---

## 8. Flight Mode Analysis

**What it measures**: Flight mode usage and transitions.

**Data source**: `vehicle_status` (nav_state field)

**Key metrics**:
- Time per mode (seconds)
- Number of mode transitions
- Mode transition timeline

**Common modes**:
| ID | Mode | Typical Use |
|----|------|------------|
| 0 | Manual | Full manual control |
| 1 | Altitude | Altitude hold, manual horizontal |
| 2 | Position | GPS position hold |
| 3 | Mission | Autonomous mission |
| 4 | Hold | Position hold (pause) |
| 5 | Return | Return to launch |
| 7 | Offboard | External control (MAVSDK/MAVLink) |
| 10 | Takeoff | Automated takeoff |
| 11 | Land | Automated landing |

**Interpretation**: Frequent mode transitions may indicate pilot uncertainty
or system instability triggering failsafe modes.

---

## 9. Overall Performance Rating

The analyzer computes an overall score (0-100) based on weighted penalties
from all metric categories:

| Component | Weight | Penalty per issue |
|-----------|--------|-------------------|
| Rate tracking | 25% | -8 to -15 per axis |
| Attitude tracking | 20% | -6 to -12 per axis |
| Position tracking | 20% | -8 to -15 |
| Vibration | 15% | -5 to -15 |
| EKF health | 10% | -7 to -15 |
| Actuator saturation | 10% | -5 to -10 |

**Grade mapping**:

| Score | Grade | Meaning |
|-------|-------|---------|
| 90-100 | A | Excellent, all metrics within range |
| 80-89 | B | Good, minor issues |
| 70-79 | C | Acceptable, some concerns |
| 60-69 | D | Poor, multiple issues need attention |
| 0-59 | F | Critical, flight safety concern |

---

## 10. Controller Comparison (INDI vs PID)

For comparing INDI and PID controller performance, run the analyzer on
flight logs from both configurations and compare:

1. **Rate tracking RMS error**: INDI should have lower error during disturbances
2. **Attitude tracking RMS error**: Both should be similar in steady state
3. **Actuator saturation**: INDI may show different saturation patterns
4. **Vibration impact**: INDI is more robust to vibration (uses angular acceleration)
5. **Disturbance recovery**: Compare settling time after step inputs

Key parameters to check:
- PID: `MC_ROLLRATE_P/I/D`, `MC_PITCHRATE_P/I/D`, `MC_YAWRATE_P/I/D`
- INDI: `MC_INDI_GAIN_P`, `MC_INDI_GAIN_Y`, `MC_INDI_FILTER`
