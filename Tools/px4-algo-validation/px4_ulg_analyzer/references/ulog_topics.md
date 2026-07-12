# PX4 ULog Key Topics Reference

This document lists the PX4 ULog topics most relevant to flight performance analysis,
organized by category. Each entry includes the topic name, key fields, typical logging
rate, and what the data is used for.

## 1. Attitude & Control

### vehicle_attitude
- **Rate**: ~250 Hz
- **Fields**: `q[0..3]` (quaternion w,x,y,z), `delta_q_reset[0..3]`, `quat_reset_counter`
- **Use**: Primary attitude data. Quaternion is the source of truth for roll/pitch/yaw.
- **Note**: In PX4 v1.14+, angular rates (rollspeed/pitchspeed/yawspeed) were moved to `vehicle_angular_velocity`. Older versions include them here.

### vehicle_angular_velocity
- **Rate**: ~250 Hz
- **Fields**: `xyz[0..2]` (angular rates in rad/s: roll, pitch, yaw), `xyz_derivative[0..2]` (angular accelerations)
- **Use**: Angular velocity data for rate tracking. Available in PX4 v1.14+.

### vehicle_attitude_setpoint
- **Rate**: ~250 Hz
- **Fields**: `q_d[0..3]` (desired quaternion), `thrust_body[0..2]`, `yaw_sp_move_rate`
- **Use**: Attitude controller setpoints. Convert q_d to euler angles for comparison against actual attitude.
- **Note**: Older PX4 versions use `roll_body`, `pitch_body`, `yaw_body` (euler angle setpoints) instead of quaternion.

### vehicle_rates_setpoint
- **Rate**: ~250 Hz
- **Fields**: `roll`, `pitch`, `yaw` (rate setpoints in rad/s), `thrust_body[3]`
- **Use**: Angular rate setpoints from the rate controller. Compare against actual rates for rate tracking performance.

### vehicle_local_position
- **Rate**: ~100 Hz
- **Fields**: `x`, `y`, `z` (NED position in meters), `vx`, `vy`, `vz` (velocity m/s), `ref_lat`, `ref_lon`, `xy_valid`, `z_valid`, `v_xy_valid`, `v_z_valid`
- **Use**: Estimated position from EKF. NED frame: x=North, y=East, z=Down (negative z = altitude).

### vehicle_local_position_setpoint
- **Rate**: ~100 Hz
- **Fields**: `x`, `y`, `z`, `vx`, `vy`, `vz`, `acc_x`, `acc_y`, `acc_z`
- **Use**: Position controller setpoints. Compare against local_position for tracking error.

### vehicle_global_position
- **Rate**: ~10 Hz
- **Fields**: `lat`, `lon`, `alt` (global WGS84), `vel_n`, `vel_e`, `vel_d`, `eph`, `epv`
- **Use**: Global position for trajectory plotting and GPS accuracy.

## 2. Sensors

### sensor_gyro
- **Rate**: ~200-800 Hz (depends on IMU)
- **Fields**: `x`, `y`, `z` (angular rates in rad/s), `temperature`, `device_id`, `clip_counter[0..2]`, `error_count`, `samples`
- **Use**: Raw gyroscope data. Used for vibration analysis (peak-to-peak, RMS, noise floor).

### sensor_accel
- **Rate**: ~200-800 Hz
- **Fields**: `x`, `y`, `z` (acceleration in m/s^2), `temperature`, `device_id`, `clip_counter[0..2]`, `error_count`, `samples`
- **Use**: Raw accelerometer data. Vibration analysis, clipping detection. High Z-axis vibration (>30 m/s^2 p2p) indicates poor mounting.

### sensor_mag
- **Rate**: ~50-80 Hz
- **Fields**: `x_ga`, `y_ga`, `z_ga`, `temperature`
- **Use**: Magnetometer data. Magnetic interference detection.

### sensor_baro
- **Rate**: ~10-50 Hz
- **Fields**: `pressure_pa`, `altitude`, `temperature`
- **Use**: Barometric pressure for altitude estimation.

## 3. Estimator (EKF2)

### estimator_status
- **Rate**: ~5-10 Hz
- **Fields**: `pos_test_ratio`, `vel_test_ratio`, `hgt_test_ratio`, `hdg_test_ratio` (heading, not mag), `tas_test_ratio`, `hagl_test_ratio`, `beta_test_ratio`, `output_tracking_error[0..2]`, `solution_status_flags`, `control_mode_flags`, `filter_fault_flags`, `pos_horiz_accuracy`, `pos_vert_accuracy`
- **Use**: EKF innovation test ratios. Values >1.0 indicate the corresponding sensor is being rejected. `output_tracking_error` indicates controller performance. Note: some fields may be NaN in SITL.

### estimator_innovations
- **Rate**: ~10 Hz
- **Fields**: `gps_hpos[0]`, `gps_hpos[1]` (GPS horizontal position), `gps_vpos` (GPS vertical), `gps_hvel[0]`, `gps_hvel[1]` (GPS horizontal velocity), `gps_vvel` (GPS vertical velocity), `baro_vpos` (baro height), `heading`, `mag_field[0..2]`, `flow[0..1]`, `drag[0..1]`, `ev_hpos[0..1]`, `ev_vpos`, `ev_hvel[0..1]`, `ev_vvel`, `airspeed`, `beta`, `hagl`, `rng_vpos`, `aux_hvel[0..1]`
- **Use**: EKF innovation values (measurement residual). Large values indicate sensor-estimator disagreement.

### estimator_event_flags
- **Rate**: event-driven
- **Fields**: `novation_check_fail`, `gps_checks_fail`, `tilt_align_complete`, `yaw_align_complete`, etc.
- **Use**: EKF initialization and fault events.

## 4. Actuators

### actuator_motors
- **Rate**: ~250 Hz
- **Fields**: `control[0..7]` (normalized -1 to 1)
- **Use**: Motor commands from mixer. Saturation detection (|control| >= 0.95).

### actuator_outputs
- **Rate**: ~250 Hz
- **Fields**: `output[0..15]` (PWM values 1000-2000 or normalized)
- **Use**: Actual PWM output to motors. Can be used if actuator_motors not available.

### actuator_controls_status_0
- **Rate**: ~250 Hz
- **Fields**: `control[0..3]` (roll, pitch, yaw, thrust normalized)
- **Use**: Normalized attitude control output before mixing.

## 5. System Status

### vehicle_status
- **Rate**: ~5 Hz
- **Fields**: `arming_state`, `nav_state` (flight mode), `vehicle_type`, `failsafe`, `hil_state`, `is_rotary_wing`, `in_transition_mode`
- **Use**: Flight mode transitions, arming state, failsafe events. PX4 flight mode IDs: 0=Manual, 1=Altitude, 2=Position, 3=Mission, 4=Hold, 5=Return, 6=Acro, 7=Offboard, 8=Stabilized, 9=Rattitude, 10=Takeoff, 11=Land.

### battery_status
- **Rate**: ~5-10 Hz
- **Fields**: `voltage_v`, `current_a`, `current_average_a`, `discharged_mah`, `remaining` (0-1), `temperature`, `cell_count`, `voltage_cell_v[0..13]`, `connected`, `warning`, `faults`, `internal_resistance_estimate`, `ocv_estimate`
- **Use**: Battery health, power consumption, discharge rate. Note: SITL logs may have `current_a = -1.0` and `discharged_mah = 0.0`.

### cpuload
- **Rate**: ~1-5 Hz
- **Fields**: `load` (0-1), `ram_usage` (0-1), `ram_total`
- **Use**: System performance. High CPU load (>80%) can cause dropped frames and control instability.

### vehicle_air_data
- **Rate**: ~10 Hz
- **Fields**: `airspeed_raw_m_s`, `airspeed_smoothed_m_s`, `baro_alt_meter`, `baro_temp_celsius`
- **Use**: Air data for fixed-wing analysis.

## 6. Parameters (for Controller Detection)

Parameters are stored in PARAMETER messages within the ULog definition section.
The message format is: `key_len(1 byte) + key_string(key_len bytes) + value(variable)`.

The key string includes the type prefix: e.g., `"float MC_INDI_ENABLE"` or `"int32_t MC_ROLLRATE_P"`.
The parser extracts the type and name from this string and decodes the value accordingly.

Key parameters for controller detection:

| Parameter | Type | Description | Values |
|-----------|------|-------------|--------|
| `MC_INDI_ENABLE` | int32 | INDI controller enable | 0=PID, 1=INDI |
| `MC_INDI_GAIN_P` | float | INDI roll/pitch gain | default 2.5 |
| `MC_INDI_GAIN_Y` | float | INDI yaw gain | default 1.5 |
| `MC_INDI_FILTER` | float | INDI acceleration filter | 0.1-0.9 |
| `MC_ROLLRATE_P/I/D` | float | PID roll rate gains | varies |
| `MC_PITCHRATE_P/I/D` | float | PID pitch rate gains | varies |
| `MC_YAWRATE_P/I/D` | float | PID yaw rate gains | varies |
| `MC_ROLL_P` | float | Roll angle P gain | varies |
| `MC_PITCH_P` | float | Pitch angle P gain | varies |
| `MC_YAW_P` | float | Yaw angle P gain | varies |
| `MPC_XY_VEL_MAX` | float | Max horizontal velocity | m/s |
| `MPC_Z_VEL_MAX_UP` | float | Max climb rate | m/s |
| `MPC_Z_VEL_MAX_DN` | float | Max descent rate | m/s |
| `MPC_XY_CRUISE` | float | Cruise horizontal speed | m/s |

## 7. PX4 Arming States

| Value | State | Description |
|-------|-------|-------------|
| 0 | DISARMED | Not armed |
| 1 | STANDBY | Pre-flight, ready to arm |
| 2 | ARMED | Armed and flying |
| 3 | STANDBY_ERROR | Error, cannot arm |
| 4 | SHUTDOWN | Shutting down |
| 5 | IN_AIR_RESTART | Restart in air |

## 8. Topic Availability Notes

Not all topics are logged in every flight. Logging depends on:
- PX4 version (newer versions log more topics)
- `sdlog2` / `logger` configuration
- Active modules (e.g., `estimator_status` only if EKF2 is running)
- SITL vs real hardware (SITL may log additional debug topics)

The analyzer handles missing topics gracefully by returning `{'available': False}` for each unavailable analysis section.
