# ULog Analysis: first flylog_335_2026-6-11-17-31-36

## Performance Rating

| Score | Grade | Summary |
|-------|-------|---------|
| **40/100** | **F** | 5 issue(s) identified, score 40/100 (grade F). |

### Issues

- 3D position tracking RMS error high: 125.67 m
- High accelerometer vibration detected
- High gyroscope vibration detected
- EKF estimator health poor (high test ratios)
- Actuator saturation moderate: 10.7%

## Flight Overview

- **Duration**: 543.2 s
- **Armed**: Yes (148.0 s)
- **Topics logged**: 83

## Controller Configuration

- **Controller type**: unknown

### Key Parameters

| Parameter | Value |
|-----------|-------|
| `MPC_XY_CRUISE` | 5.0 |

## Attitude Statistics

| Axis | Mean | Std | Min | Max |
|------|------|-----|-----|-----|
| Roll | 4.4 deg | 12.96 deg | -29.2 deg | 39.1 deg |
| Pitch | -0.2 deg | 4.00 deg | -8.0 deg | 13.9 deg |
| Yaw | -19.2 deg | 79.83 deg | -179.8 deg | 179.5 deg |

### Angular Rates

| Axis | Mean (rad/s) | RMS (rad/s) | Max (rad/s) |
|------|-------------|-------------|-------------|
| Roll | 0.0015 | 0.2045 | 1.7132 |
| Pitch | 0.0276 | 0.0892 | 1.9358 |
| Yaw | 0.0400 | 0.1389 | 0.7939 |

## Rate Tracking Performance

| Axis | RMS Error (deg/s) | Mean Error (deg/s) | Max Error (deg/s) |
|------|-------------------|--------------------|--------------------|
| Roll | 0.159 | 0.104 | 1.752 |
| Pitch | 0.101 | 0.059 | 1.809 |
| Yaw | 0.096 | 0.052 | 0.738 |

## Attitude Tracking Performance

| Axis | RMS Error (deg) | Mean Error (deg) | Max Error (deg) |
|------|-----------------|------------------|------------------|
| Roll | 3.050 | 1.670 | 24.770 |
| Pitch | 1.930 | 1.190 | 8.540 |
| Yaw | 10.630 | 1.890 | 359.490 |

## Position Tracking Performance

- **3D RMS Error**: 125.674 m
- **3D Max Error**: 445.256 m

| Axis | RMS Error (m) | Max Error (m) |
|------|--------------|--------------|
| X | 73.0768 | 342.9390 |
| Y | 98.5907 | 398.7704 |
| Z | 27.0844 | 50.0666 |

- **Altitude range**: 2.0 ~ 55.8 m

## Vibration Analysis

- **Gyroscope level**: high
- **Accelerometer level**: high

### Gyroscope Vibration (rad/s)

| Axis | Peak-to-Peak | RMS | Std |
|------|-------------|-----|-----|
| X | 1.8958 | 0.2005 | 0.2005 |
| Y | 0.9436 | 0.1069 | 0.1062 |
| Z | 0.9089 | 0.1450 | 0.1397 |

### Accelerometer Vibration (m/s^2)

| Axis | Peak-to-Peak | RMS | Std |
|------|-------------|-----|-----|
| X | 19.61 | 1.56 | 1.54 |
| Y | 25.90 | 1.83 | 1.79 |
| Z | 40.47 | 10.94 | 3.66 |

## Estimator (EKF) Performance

- **Health**: poor

| Source | Mean | Max | P95 |
|--------|------|-----|-----|
| Position | 0.0347 | 0.1424 | 0.0852 |
| Velocity | 0.0646 | 2.4703 | 0.1608 |
| Magnetometer | 0.1221 | 0.4545 | 0.3110 |
| Height | 0.0194 | 0.0922 | 0.0567 |

### Innovations

| Source | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| gps_hpos_x | -0.0196 | 0.0902 | -0.2348 | 0.3501 |
| gps_hpos_y | -0.0076 | 0.0820 | -0.3166 | 0.2809 |
| gps_vpos | -0.0044 | 0.1359 | -0.4379 | 0.5016 |
| gps_hvel_x | -0.0206 | 0.1331 | -2.5512 | 0.2742 |
| gps_hvel_y | -0.0056 | 0.0894 | -0.5715 | 0.5124 |
| gps_vvel | -0.0105 | 0.2841 | -5.5931 | 0.5544 |
| baro_vpos | 0.0073 | 0.3720 | -1.0440 | 1.4399 |
| heading | 0.0081 | 0.0121 | -0.0037 | 0.0713 |
| mag_x | 0.0016 | 0.0066 | -0.0236 | 0.0375 |
| mag_y | 0.0027 | 0.0052 | -0.0132 | 0.0221 |
| mag_z | -0.0093 | 0.0219 | -0.0713 | 0.0713 |

## Battery & Power

- **Voltage**: 24.1V -> 23.9V (drop 0.2V)
- **Voltage range**: 22.4V ~ 24.2V
- **Current**: mean 0.0A, max 0.5A
- **Energy consumed**: 2 mAh
- **Power**: mean 0.5W, max 12.3W
- **Remaining**: 92.5% -> 79.3%

## Flight Mode Analysis

- **Transitions**: 5
- **Modes used**: Unknown(15), Mission, Manual

### Time per Mode

| Mode | Duration (s) |
|------|-------------|
| Mission | 120.9 |
| Unknown(15) | 27.1 |
| Manual | 0.9 |

### Mode Transitions

| Time (s) | From | To |
|----------|------|-----|
| 415.6 | Unknown(15) | Manual |
| 415.7 | Manual | Unknown(15) |
| 417.6 | Unknown(15) | Mission |
| 538.4 | Mission | Manual |
| 539.2 | Manual | Unknown(15) |

## Actuator Analysis

- **Source**: actuator_motors
- **Total saturation**: 10.7%

| Motor | Mean | Min | Max | Saturation % |
|-------|------|-----|-----|-------------|
| motor_0 | 0.6307 | 0.0320 | 1.0000 | 10.7% |

## Logged Topics

| Topic | Samples |
|-------|---------|
| `action_request` | 6 |
| `actuator_armed` | 303 |
| `actuator_motors` | 1491 |
| `actuator_outputs` | 1491 |
| `actuator_servos` | 1491 |
| `airspeed` | 148 |
| `airspeed_validated` | 746 |
| `airspeed_wind` | 296 |
| `battery_status` | 746 |
| `config_overrides` | 298 |
| `control_allocator_status` | 746 |
| `cpuload` | 298 |
| `differential_pressure` | 149 |
| `estimator_attitude` | 594 |
| `estimator_baro_bias` | 595 |
| `estimator_event_flags` | 299 |
| `estimator_global_position` | 296 |
| `estimator_gps_status` | 298 |
| `estimator_innovation_test_ratios` | 596 |
| `estimator_innovation_variances` | 596 |
| `estimator_innovations` | 596 |
| `estimator_local_position` | 594 |
| `estimator_selector_status` | 149 |
| `estimator_sensor_bias` | 451 |
| `estimator_states` | 298 |
| `estimator_status` | 1492 |
| `estimator_status_flags` | 312 |
| `estimator_wind` | 246 |
| `event` | 22 |
| `failsafe_flags` | 273 |
| `failure_detector_status` | 303 |
| `flaps_setpoint` | 148 |
| `home_position` | 8 |
| `input_rc` | 297 |
| `landing_gear` | 2 |
| `magnetometer_bias_estimate` | 7 |
| `manual_control_setpoint` | 746 |
| `manual_control_switches` | 145 |
| `mission_result` | 7 |
| `navigator_mission_item` | 6 |
| `npfg_status` | 1211 |
| `parameter_update` | 2 |
| `position_controller_landing_status` | 357 |
| `position_controller_status` | 244 |
| `position_setpoint_triplet` | 10 |
| `px4io_status` | 150 |
| `rate_ctrl_status` | 745 |
| `rtl_status` | 74 |
| `rtl_time_estimate` | 74 |
| `sensor_accel` | 298 |
| `sensor_baro` | 149 |
| `sensor_combined` | 29686 |
| `sensor_gps` | 149 |
| `sensor_gyro` | 298 |
| `sensor_gyro_fft` | 1893 |
| `sensor_mag` | 298 |
| `sensors_status_imu` | 746 |
| `spoilers_setpoint` | 148 |
| `system_power` | 297 |
| `tecs_status` | 607 |
| `telemetry_status` | 148 |
| `vehicle_acceleration` | 2980 |
| `vehicle_air_data` | 746 |
| `vehicle_angular_velocity` | 7447 |
| `vehicle_attitude` | 2980 |
| `vehicle_attitude_setpoint` | 2966 |
| `vehicle_command` | 9 |
| `vehicle_command_ack` | 3 |
| `vehicle_control_mode` | 303 |
| `vehicle_global_position` | 746 |
| `vehicle_gps_position` | 746 |
| `vehicle_imu` | 596 |
| `vehicle_imu_status` | 298 |
| `vehicle_land_detected` | 153 |
| `vehicle_local_position` | 1491 |
| `vehicle_local_position_setpoint` | 1211 |
| `vehicle_magnetometer` | 297 |
| `vehicle_rates_setpoint` | 7406 |
| `vehicle_status` | 303 |
| `vehicle_thrust_setpoint` | 7447 |
| `vehicle_torque_setpoint` | 7447 |
| `wind` | 123 |
| `yaw_estimator_status` | 298 |
