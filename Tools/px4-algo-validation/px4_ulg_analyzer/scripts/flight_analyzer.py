"""
flight_analyzer.py - Comprehensive PX4 flight performance analyzer

Computes flight performance metrics from parsed ULog data:
  - Flight info (duration, topics, arming state)
  - Attitude statistics (roll/pitch/yaw from quaternion)
  - Rate tracking (setpoint vs actual angular rates)
  - Attitude tracking (setpoint vs actual euler angles)
  - Position tracking (setpoint vs estimated position)
  - Vibration analysis (sensor gyro/accel peak-to-peak, RMS)
  - Estimator/EKF performance (innovations, test ratios)
  - Battery & power analysis
  - Flight mode transitions
  - Actuator saturation analysis
  - Controller configuration detection (INDI vs PID)
  - Overall performance rating

Usage:
    from ulog_parser import ULogParser
    from flight_analyzer import FlightPerformanceAnalyzer

    ulog = ULogParser("flight.ulg").parse()
    analyzer = FlightPerformanceAnalyzer(ulog)
    results = analyzer.analyze()
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any

from ulog_parser import ULogFile, TopicData


# ============================================================
# Helper Functions
# ============================================================

def _is_valid(v: Any) -> bool:
    """Check if a value is a valid number (not NaN, not None)."""
    if v is None:
        return False
    if isinstance(v, float) and math.isnan(v):
        return False
    if isinstance(v, float) and math.isinf(v):
        return False
    return True


def _filter_valid(values: List[float]) -> List[float]:
    """Filter out NaN, Inf, and None values from a list."""
    return [v for v in values if _is_valid(v)]


def _mean(values: List[float]) -> float:
    values = _filter_valid(values)
    if not values:
        return 0.0
    return sum(values) / len(values)


def _std(values: List[float]) -> float:
    values = _filter_valid(values)
    if len(values) < 2:
        return 0.0
    m = sum(values) / len(values)
    return math.sqrt(sum((x - m) ** 2 for x in values) / len(values))


def _rms(values: List[float]) -> float:
    values = _filter_valid(values)
    if not values:
        return 0.0
    return math.sqrt(sum(x * x for x in values) / len(values))


def _percentile(values: List[float], p: float) -> float:
    """Calculate p-th percentile (0-100)"""
    values = _filter_valid(values)
    if not values:
        return 0.0
    sorted_vals = sorted(values)
    k = (len(sorted_vals) - 1) * p / 100.0
    f = int(math.floor(k))
    c = int(math.ceil(k))
    if f == c:
        return sorted_vals[int(k)]
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


def _stats(values: List[float]) -> Dict[str, float]:
    """Compute standard statistics dict, filtering NaN/Inf values."""
    values = _filter_valid(values)
    if not values:
        return {}
    return {
        'mean': round(sum(values) / len(values), 4),
        'min': round(min(values), 4),
        'max': round(max(values), 4),
        'std': round(_std(values), 4),
        'rms': round(_rms(values), 4),
        'p95': round(_percentile(values, 95), 4),
    }


def _quat_to_euler(q: List[float]) -> Tuple[float, float, float]:
    """Quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw) in radians"""
    w, x, y, z = q[0], q[1], q[2], q[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _align_by_timestamp(setpoint_data: List[TopicData],
                        actual_data: List[TopicData],
                        field_sp: str,
                        field_actual: str) -> Tuple[List[float], List[float]]:
    """Align two time series by nearest timestamp for error calculation.

    Returns (setpoint_values, actual_values) with matched timestamps.
    """
    if not setpoint_data or not actual_data:
        return [], []

    sp_vals = [(d.timestamp, d.values.get(field_sp, 0)) for d in setpoint_data if field_sp in d.values]
    act_vals = [(d.timestamp, d.values.get(field_actual, 0)) for d in actual_data if field_actual in d.values]

    if not sp_vals or not act_vals:
        return [], []

    # Use actual data timestamps as reference, find nearest setpoint
    sp_idx = 0
    matched_sp = []
    matched_act = []

    for act_ts, act_v in act_vals:
        # Advance setpoint index to be closest to actual timestamp
        while sp_idx < len(sp_vals) - 1 and abs(sp_vals[sp_idx + 1][0] - act_ts) < abs(sp_vals[sp_idx][0] - act_ts):
            sp_idx += 1
        matched_sp.append(sp_vals[sp_idx][1])
        matched_act.append(act_v)

    return matched_sp, matched_act


# ============================================================
# Flight Performance Rating
# ============================================================

@dataclass
class PerformanceRating:
    """Overall flight performance assessment"""
    score: int                    # 0-100
    grade: str                    # A/B/C/D/F
    summary: str                  # One-line summary
    issues: List[str]             # Identified issues


# ============================================================
# Main Analyzer
# ============================================================

class FlightPerformanceAnalyzer:
    """Comprehensive PX4 flight performance analyzer.

    Analyzes a parsed ULogFile and produces a structured results dict
    with all performance metrics.
    """

    # PX4 flight mode names (subset, most common)
    FLIGHT_MODES = {
        0: 'Manual', 1: 'Altitude', 2: 'Position',
        3: 'Mission', 4: 'Hold', 5: 'Return',
        6: 'Acro', 7: 'Offboard', 8: 'Stabilized',
        9: 'Rattitude', 10: 'Takeoff', 11: 'Land',
        12: 'Follow Target', 13: 'Precision Land',
    }

    # Vibration thresholds (m/s^2 for accel, rad/s for gyro)
    VIB_ACCEL_GOOD = 15.0      # <15 m/s^2 peak-to-peak is good
    VIB_ACCEL_WARN = 30.0      # 15-30 is acceptable, >30 is concerning
    VIB_GYRO_GOOD = 0.2        # rad/s peak-to-peak
    VIB_GYRO_WARN = 0.5

    def __init__(self, ulog: ULogFile):
        self.ulog = ulog
        self._t0 = ulog.start_timestamp_us

    def analyze(self) -> dict:
        """Run all analyses and return structured results."""
        return {
            'flight_info': self._analyze_flight_info(),
            'attitude': self._analyze_attitude(),
            'rate_tracking': self._analyze_rate_tracking(),
            'attitude_tracking': self._analyze_attitude_tracking(),
            'position_tracking': self._analyze_position_tracking(),
            'vibration': self._analyze_vibration(),
            'estimator': self._analyze_estimator(),
            'battery': self._analyze_battery(),
            'flight_modes': self._analyze_flight_modes(),
            'actuators': self._analyze_actuators(),
            'controller_config': self._analyze_controller_config(),
            'performance_rating': None,  # Filled after all analyses
        }

    def analyze_with_rating(self) -> dict:
        """Run all analyses and compute overall performance rating."""
        results = self.analyze()
        results['performance_rating'] = self._rate_performance(results)
        return results

    # ----------------------------------------------------------
    # 1. Flight Info
    # ----------------------------------------------------------

    def _analyze_flight_info(self) -> dict:
        """Basic flight information"""
        duration = self.ulog.duration_s

        # Arming state
        armed = False
        armed_time = 0.0
        vehicle_status = self.ulog.get_topic('vehicle_status')
        if vehicle_status:
            armed_periods = []
            arm_start = None
            prev_armed = False
            for d in vehicle_status:
                is_armed = d.values.get('arming_state', 0) == 2  # PX4 ARMING_STATE_ARMED = 2
                if is_armed and not prev_armed:
                    arm_start = d.timestamp
                elif not is_armed and prev_armed and arm_start is not None:
                    armed_periods.append((arm_start, d.timestamp))
                    arm_start = None
                prev_armed = is_armed
            if arm_start is not None:
                armed_periods.append((arm_start, vehicle_status[-1].timestamp))

            armed = len(armed_periods) > 0
            armed_time = sum((e - s) / 1e6 for s, e in armed_periods)

        return {
            'duration_s': round(duration, 2),
            'armed': armed,
            'armed_time_s': round(armed_time, 2),
            'start_time_us': self.ulog.start_timestamp_us,
            'end_time_us': self.ulog.end_timestamp_us,
            'topics_recorded': sorted(self.ulog.topics.keys()),
            'topic_count': len(self.ulog.topics),
            'topic_counts': {k: len(v) for k, v in self.ulog.topics.items()},
            'info': dict(self.ulog.info),
        }

    # ----------------------------------------------------------
    # 2. Attitude
    # ----------------------------------------------------------

    def _analyze_attitude(self) -> dict:
        """Attitude statistics from vehicle_attitude (quaternion) + vehicle_angular_velocity (rates).

        PX4 v1.14+ splits angular rates into vehicle_angular_velocity topic.
        Older versions include rollspeed/pitchspeed/yawspeed in vehicle_attitude.
        """
        data = self.ulog.get_topic('vehicle_attitude')
        if not data:
            return {}

        rolls, pitches, yaws = [], [], []

        for entry in data:
            q = [entry.values.get(f'q[{i}]', 0) for i in range(4)]
            if not all(_is_valid(v) for v in q):
                continue
            roll, pitch, yaw = _quat_to_euler(q)
            rolls.append(math.degrees(roll))
            pitches.append(math.degrees(pitch))
            yaws.append(math.degrees(yaw))

        # Angular rates: try vehicle_angular_velocity first, then vehicle_attitude
        rate_data = self.ulog.get_topic('vehicle_angular_velocity')
        roll_rates, pitch_rates, yaw_rates = [], [], []

        if rate_data:
            for entry in rate_data:
                r = entry.values.get('xyz[0]')
                p = entry.values.get('xyz[1]')
                y = entry.values.get('xyz[2]')
                if _is_valid(r):
                    roll_rates.append(r)
                if _is_valid(p):
                    pitch_rates.append(p)
                if _is_valid(y):
                    yaw_rates.append(y)
        else:
            # Fallback: older PX4 versions have rates in vehicle_attitude
            for entry in data:
                r = entry.values.get('rollspeed')
                p = entry.values.get('pitchspeed')
                y = entry.values.get('yawspeed')
                if _is_valid(r):
                    roll_rates.append(r)
                if _is_valid(p):
                    pitch_rates.append(p)
                if _is_valid(y):
                    yaw_rates.append(y)

        return {
            'euler_deg': {
                'roll': _stats(rolls),
                'pitch': _stats(pitches),
                'yaw': _stats(yaws),
            },
            'angular_rate_rad_s': {
                'roll': _stats(roll_rates),
                'pitch': _stats(pitch_rates),
                'yaw': _stats(yaw_rates),
            },
            'sample_count': len(data),
            'sample_rate_hz': round(len(data) / max(self.ulog.duration_s, 0.001), 1),
        }

    # ----------------------------------------------------------
    # 3. Rate Tracking
    # ----------------------------------------------------------

    def _analyze_rate_tracking(self) -> dict:
        """Angular rate tracking: setpoint vs actual.

        Compares vehicle_rates_setpoint (roll/pitch/yaw) against
        vehicle_angular_velocity (xyz[0..2]) or vehicle_attitude (rollspeed etc.).
        """
        setpoint_data = self.ulog.get_topic('vehicle_rates_setpoint')

        # Find actual rate source: vehicle_angular_velocity (new) or vehicle_attitude (old)
        actual_data = self.ulog.get_topic('vehicle_angular_velocity')
        rate_source = 'vehicle_angular_velocity'
        actual_fields = ('xyz[0]', 'xyz[1]', 'xyz[2]')

        if not actual_data:
            actual_data = self.ulog.get_topic('vehicle_attitude')
            rate_source = 'vehicle_attitude'
            actual_fields = ('rollspeed', 'pitchspeed', 'yawspeed')

        if not setpoint_data or not actual_data:
            return {'available': False, 'reason': 'Missing vehicle_rates_setpoint or rate source'}

        result = {
            'available': True,
            'sample_count': len(setpoint_data),
            'rate_source': rate_source,
        }
        axes = [
            ('roll', 'roll', actual_fields[0]),
            ('pitch', 'pitch', actual_fields[1]),
            ('yaw', 'yaw', actual_fields[2]),
        ]

        for axis_name, sp_field, actual_field in axes:
            sp_vals, act_vals = _align_by_timestamp(
                setpoint_data, actual_data, sp_field, actual_field)

            # Filter NaN
            sp_vals = _filter_valid(sp_vals)
            act_vals = _filter_valid(act_vals)

            if not sp_vals or not act_vals:
                result[axis_name] = {}
                continue

            min_len = min(len(sp_vals), len(act_vals))
            sp_vals = sp_vals[:min_len]
            act_vals = act_vals[:min_len]

            errors = [s - a for s, a in zip(sp_vals, act_vals)]
            abs_errors = [abs(e) for e in errors]

            result[axis_name] = {
                'setpoint_stats': _stats(sp_vals),
                'actual_stats': _stats(act_vals),
                'error_stats': _stats(errors),
                'abs_error': {
                    'mean_deg_s': round(_mean(abs_errors), 4),
                    'max_deg_s': round(max(abs_errors), 4),
                    'rms_deg_s': round(_rms(abs_errors), 4),
                },
            }

        return result

    # ----------------------------------------------------------
    # 4. Attitude Tracking
    # ----------------------------------------------------------

    def _analyze_attitude_tracking(self) -> dict:
        """Attitude tracking: setpoint vs actual euler angles.

        Compares vehicle_attitude_setpoint quaternion (q_d[0..3]) against
        vehicle_attitude quaternion (q[0..3]), both converted to euler angles.
        Falls back to roll_body/pitch_body/yaw_body if available (older PX4).
        """
        setpoint_data = self.ulog.get_topic('vehicle_attitude_setpoint')
        actual_data = self.ulog.get_topic('vehicle_attitude')

        if not setpoint_data or not actual_data:
            return {'available': False, 'reason': 'Missing vehicle_attitude_setpoint or vehicle_attitude'}

        # Pre-compute actual euler angles
        actual_euler = []
        for entry in actual_data:
            q = [entry.values.get(f'q[{i}]', 0) for i in range(4)]
            if not all(_is_valid(v) for v in q):
                continue
            roll, pitch, yaw = _quat_to_euler(q)
            actual_euler.append({
                'timestamp': entry.timestamp,
                'roll': math.degrees(roll),
                'pitch': math.degrees(pitch),
                'yaw': math.degrees(yaw),
            })

        result = {'available': True, 'sample_count': len(setpoint_data)}

        # Check if setpoint has euler fields (older PX4) or quaternion (newer)
        has_euler = any('roll_body' in d.values for d in setpoint_data[:5])

        if has_euler:
            sp_fields = [('roll', 'roll_body'), ('pitch', 'pitch_body'), ('yaw', 'yaw_body')]
        else:
            sp_fields = [('roll', 'q_d'), ('pitch', 'q_d'), ('yaw', 'q_d')]

        for axis_name, sp_field in sp_fields:
            if sp_field == 'q_d':
                # Convert setpoint quaternion to euler
                sp_ts = []
                sp_vals_deg = []
                for d in setpoint_data:
                    q = [d.values.get(f'q_d[{i}]', 0) for i in range(4)]
                    if not all(_is_valid(v) for v in q):
                        continue
                    roll, pitch, yaw = _quat_to_euler(q)
                    sp_ts.append(d.timestamp)
                    if axis_name == 'roll':
                        sp_vals_deg.append(math.degrees(roll))
                    elif axis_name == 'pitch':
                        sp_vals_deg.append(math.degrees(pitch))
                    else:
                        sp_vals_deg.append(math.degrees(yaw))
            else:
                sp_ts = [d.timestamp for d in setpoint_data if sp_field in d.values and _is_valid(d.values.get(sp_field))]
                sp_vals_deg = [math.degrees(d.values[sp_field]) for d in setpoint_data
                               if sp_field in d.values and _is_valid(d.values.get(sp_field))]

            if not sp_vals_deg:
                result[axis_name] = {}
                continue

            # Align by nearest timestamp
            act_vals_deg = []
            sp_idx = 0
            for ae in actual_euler:
                while sp_idx < len(sp_ts) - 1 and abs(sp_ts[sp_idx + 1] - ae['timestamp']) < abs(sp_ts[sp_idx] - ae['timestamp']):
                    sp_idx += 1
                act_vals_deg.append(ae[axis_name])

            min_len = min(len(sp_vals_deg), len(act_vals_deg))
            sp_matched = sp_vals_deg[:min_len]
            act_matched = act_vals_deg[:min_len]

            errors = [s - a for s, a in zip(sp_matched, act_matched)]
            abs_errors = [abs(e) for e in errors]

            result[axis_name] = {
                'setpoint_stats': _stats(sp_matched),
                'actual_stats': _stats(act_matched),
                'error_stats': _stats(errors),
                'abs_error': {
                    'mean_deg': round(_mean(abs_errors), 2),
                    'max_deg': round(max(abs_errors), 2),
                    'rms_deg': round(_rms(abs_errors), 2),
                },
            }

        return result

    # ----------------------------------------------------------
    # 5. Position Tracking
    # ----------------------------------------------------------

    def _analyze_position_tracking(self) -> dict:
        """Position tracking: setpoint vs estimated local position.

        Compares vehicle_local_position_setpoint (x, y, z) against
        vehicle_local_position (x, y, z) in NED frame.
        """
        setpoint_data = self.ulog.get_topic('vehicle_local_position_setpoint')
        actual_data = self.ulog.get_topic('vehicle_local_position')

        if not setpoint_data or not actual_data:
            return {'available': False, 'reason': 'Missing position topics'}

        result = {'available': True, 'sample_count': len(actual_data)}

        for axis in ['x', 'y', 'z']:
            sp_vals, act_vals = _align_by_timestamp(
                setpoint_data, actual_data, axis, axis)

            # Filter NaN values
            sp_vals = _filter_valid(sp_vals)
            act_vals = _filter_valid(act_vals)

            if not sp_vals or not act_vals:
                result[axis] = {}
                continue

            min_len = min(len(sp_vals), len(act_vals))
            sp_vals = sp_vals[:min_len]
            act_vals = act_vals[:min_len]

            errors = [s - a for s, a in zip(sp_vals, act_vals)]
            abs_errors = [abs(e) for e in errors]

            result[axis] = {
                'setpoint_stats': _stats(sp_vals),
                'actual_stats': _stats(act_vals),
                'error_stats': _stats(errors),
                'abs_error': {
                    'mean_m': round(_mean(abs_errors), 4),
                    'max_m': round(max(abs_errors), 4),
                    'rms_m': round(_rms(abs_errors), 4),
                },
            }

        # Calculate 3D position error
        x_sp, x_act = _align_by_timestamp(setpoint_data, actual_data, 'x', 'x')
        y_sp, y_act = _align_by_timestamp(setpoint_data, actual_data, 'y', 'y')
        z_sp, z_act = _align_by_timestamp(setpoint_data, actual_data, 'z', 'z')

        # Filter NaN values
        x_sp, x_act = _filter_valid(x_sp), _filter_valid(x_act)
        y_sp, y_act = _filter_valid(y_sp), _filter_valid(y_act)
        z_sp, z_act = _filter_valid(z_sp), _filter_valid(z_act)

        if x_sp and y_sp and z_sp and x_act and y_act and z_act:
            min_len = min(len(x_sp), len(y_sp), len(z_sp), len(x_act), len(y_act), len(z_act))
            errors_3d = []
            for i in range(min_len):
                dx = x_sp[i] - x_act[i]
                dy = y_sp[i] - y_act[i]
                dz = z_sp[i] - z_act[i]
                err = math.sqrt(dx * dx + dy * dy + dz * dz)
                if _is_valid(err):
                    errors_3d.append(err)

            result['error_3d'] = {
                'mean_m': round(_mean(errors_3d), 4),
                'max_m': round(max(errors_3d), 4),
                'rms_m': round(_rms(errors_3d), 4),
            }

        # Altitude info
        alt_data = [d.values.get('z', 0) for d in actual_data if 'z' in d.values and _is_valid(d.values.get('z'))]
        if alt_data:
            result['altitude'] = {
                'min_m': round(-max(alt_data), 2),  # NED z is negative up
                'max_m': round(-min(alt_data), 2),
                'mean_m': round(-_mean(alt_data), 2),
            }

        return result

    # ----------------------------------------------------------
    # 6. Vibration Analysis
    # ----------------------------------------------------------

    def _analyze_vibration(self) -> dict:
        """Vibration analysis from raw sensor data.

        Computes peak-to-peak and RMS of gyro and accel signals.
        High vibration indicates mechanical issues or poor isolation.
        """
        result = {'available': False}

        # Gyro vibration
        gyro = self.ulog.get_topic('sensor_gyro')
        if gyro:
            result['available'] = True
            gyro_result = {}
            for axis in ['x', 'y', 'z']:
                # Try multiple field name patterns
                key = axis  # PX4: just 'x', 'y', 'z'
                vals = [d.values.get(key, 0) for d in gyro if key in d.values and _is_valid(d.values.get(key))]
                if not vals:
                    key = f'{axis}_rad_s'  # older format
                    vals = [d.values.get(key, 0) for d in gyro if key in d.values and _is_valid(d.values.get(key))]
                if vals:
                    p2p = max(vals) - min(vals)
                    gyro_result[axis] = {
                        'peak_to_peak_rad_s': round(p2p, 4),
                        'rms_rad_s': round(_rms(vals), 4),
                        'std_rad_s': round(_std(vals), 4),
                    }
            result['gyro'] = gyro_result

            # Overall vibration assessment
            all_p2p = []
            for axis_data in gyro_result.values():
                if 'peak_to_peak_rad_s' in axis_data:
                    all_p2p.append(axis_data['peak_to_peak_rad_s'])
            if all_p2p:
                max_p2p = max(all_p2p)
                if max_p2p < self.VIB_GYRO_GOOD:
                    result['gyro_level'] = 'good'
                elif max_p2p < self.VIB_GYRO_WARN:
                    result['gyro_level'] = 'acceptable'
                else:
                    result['gyro_level'] = 'high'

        # Accel vibration
        accel = self.ulog.get_topic('sensor_accel')
        if accel:
            result['available'] = True
            accel_result = {}
            for axis in ['x', 'y', 'z']:
                key = axis  # PX4: just 'x', 'y', 'z'
                vals = [d.values.get(key, 0) for d in accel if key in d.values and _is_valid(d.values.get(key))]
                if not vals:
                    key = f'{axis}_m_s2'  # older format
                    vals = [d.values.get(key, 0) for d in accel if key in d.values and _is_valid(d.values.get(key))]
                if vals:
                    p2p = max(vals) - min(vals)
                    accel_result[axis] = {
                        'peak_to_peak_m_s2': round(p2p, 4),
                        'rms_m_s2': round(_rms(vals), 4),
                        'std_m_s2': round(_std(vals), 4),
                    }
            result['accel'] = accel_result

            all_p2p = []
            for axis_data in accel_result.values():
                if 'peak_to_peak_m_s2' in axis_data:
                    all_p2p.append(axis_data['peak_to_peak_m_s2'])
            if all_p2p:
                max_p2p = max(all_p2p)
                if max_p2p < self.VIB_ACCEL_GOOD:
                    result['accel_level'] = 'good'
                elif max_p2p < self.VIB_ACCEL_WARN:
                    result['accel_level'] = 'acceptable'
                else:
                    result['accel_level'] = 'high'

        if not result['available']:
            result['reason'] = 'No sensor_gyro or sensor_accel data found'

        return result

    # ----------------------------------------------------------
    # 7. Estimator/EKF Performance
    # ----------------------------------------------------------

    def _analyze_estimator(self) -> dict:
        """EKF estimator performance from estimator_status topic."""
        data = self.ulog.get_topic('estimator_status')
        if not data:
            return {'available': False, 'reason': 'No estimator_status topic'}

        result = {
            'available': True,
            'sample_count': len(data),
        }

        # Key EKF test ratios (field names vary by PX4 version)
        metrics_fields = [
            ('pos_test_ratio', 'pos_test_ratio'),
            ('vel_test_ratio', 'vel_test_ratio'),
            ('hgt_test_ratio', 'hgt_test_ratio'),
            ('hdg_test_ratio', 'hdg_test_ratio'),      # heading (newer PX4)
            ('mag_test_ratio', 'mag_test_ratio'),       # magnetometer (older PX4)
            ('tas_test_ratio', 'tas_test_ratio'),
            ('hagl_test_ratio', 'hagl_test_ratio'),
            ('beta_test_ratio', 'beta_test_ratio'),
        ]

        for field, label in metrics_fields:
            vals = [d.values.get(field, float('nan')) for d in data if field in d.values]
            vals = _filter_valid(vals)
            if vals:
                result[label] = {
                    'mean': round(_mean(vals), 4),
                    'max': round(max(vals), 4),
                    'p95': round(_percentile(vals, 95), 4),
                }

        # Output tracking error (controller performance indicator in EKF)
        for i in range(3):
            field = f'output_tracking_error[{i}]'
            vals = [d.values.get(field, float('nan')) for d in data if field in d.values]
            vals = _filter_valid(vals)
            if vals:
                result[f'output_tracking_error_{i}'] = _stats(vals)

        # Innovation data
        innov_data = self.ulog.get_topic('estimator_innovations')
        if innov_data:
            # PX4 innovation field names (v1.14+)
            innov_fields = [
                ('gps_hpos_x', 'gps_hpos[0]'),
                ('gps_hpos_y', 'gps_hpos[1]'),
                ('gps_vpos', 'gps_vpos'),
                ('gps_hvel_x', 'gps_hvel[0]'),
                ('gps_hvel_y', 'gps_hvel[1]'),
                ('gps_vvel', 'gps_vvel'),
                ('baro_vpos', 'baro_vpos'),
                ('heading', 'heading'),
                ('mag_x', 'mag_field[0]'),
                ('mag_y', 'mag_field[1]'),
                ('mag_z', 'mag_field[2]'),
            ]
            # Also try older field names
            innov_fields_old = [
                ('pos_x', 'pos_x_innov'),
                ('pos_y', 'pos_y_innov'),
                ('pos_z', 'pos_z_innov'),
                ('vel_x', 'vel_x_innov'),
                ('vel_y', 'vel_y_innov'),
                ('vel_z', 'vel_z_innov'),
            ]

            innovations = {}
            for label, field in innov_fields + innov_fields_old:
                vals = [d.values.get(field, float('nan')) for d in innov_data if field in d.values]
                vals = _filter_valid(vals)
                if vals:
                    innovations[label] = _stats(vals)
            if innovations:
                result['innovations'] = innovations

        # Overall EKF health assessment
        # Use pos and vel test ratios if available, fall back to hgt
        pos_ratio = result.get('pos_test_ratio', {}).get('max', 0)
        vel_ratio = result.get('vel_test_ratio', {}).get('max', 0)
        hgt_ratio = result.get('hgt_test_ratio', {}).get('max', 0)

        max_ratio = max(pos_ratio, vel_ratio, hgt_ratio)
        if max_ratio > 1.0:
            result['health'] = 'poor'
        elif max_ratio > 0.5:
            result['health'] = 'marginal'
        else:
            result['health'] = 'good'

        return result

    # ----------------------------------------------------------
    # 8. Battery & Power
    # ----------------------------------------------------------

    def _analyze_battery(self) -> dict:
        """Battery and power consumption analysis."""
        data = self.ulog.get_topic('battery_status')
        if not data:
            return {'available': False, 'reason': 'No battery_status topic'}

        result = {'available': True, 'sample_count': len(data)}

        # Voltage
        voltages = [d.values.get('voltage_v', 0) for d in data if 'voltage_v' in d.values and _is_valid(d.values.get('voltage_v'))]
        if voltages:
            result['voltage_v'] = {
                'start': round(voltages[0], 2),
                'end': round(voltages[-1], 2),
                'min': round(min(voltages), 2),
                'max': round(max(voltages), 2),
                'mean': round(_mean(voltages), 2),
            }
            result['voltage_drop_v'] = round(voltages[0] - voltages[-1], 2)

        # Current
        currents = [d.values.get('current_a', 0) for d in data if 'current_a' in d.values and _is_valid(d.values.get('current_a'))]
        if currents:
            result['current_a'] = {
                'mean': round(_mean(currents), 2),
                'max': round(max(currents), 2),
                'min': round(min(currents), 2),
            }

        # Consumed charge - try both field names
        consumed = [d.values.get('consumed_mah', d.values.get('discharged_mah', 0))
                    for d in data if ('consumed_mah' in d.values or 'discharged_mah' in d.values)]
        consumed = _filter_valid(consumed)
        if consumed:
            result['consumed_mah'] = round(consumed[-1], 1)

        # Remaining
        remaining = [d.values.get('remaining', 0) for d in data if 'remaining' in d.values and _is_valid(d.values.get('remaining'))]
        if remaining:
            result['remaining_pct'] = {
                'start': round(remaining[0] * 100, 1),
                'end': round(remaining[-1] * 100, 1),
            }

        # Power estimation
        if voltages and currents:
            power = [v * c for v, c in zip(voltages, currents)]
            result['power_w'] = {
                'mean': round(_mean(power), 1),
                'max': round(max(power), 1),
            }

        return result

    # ----------------------------------------------------------
    # 9. Flight Mode Analysis
    # ----------------------------------------------------------

    def _analyze_flight_modes(self) -> dict:
        """Flight mode transitions and time per mode."""
        data = self.ulog.get_topic('vehicle_status')
        if not data:
            return {'available': False, 'reason': 'No vehicle_status topic'}

        result = {'available': True, 'sample_count': len(data)}

        # Extract mode sequence
        modes = []
        for d in data:
            nav_state = d.values.get('nav_state', d.values.get('nav_state', 0))
            ts = d.timestamp
            modes.append((ts, nav_state))

        if not modes:
            return result

        # Calculate time per mode
        mode_times: Dict[int, float] = {}
        mode_names: Dict[int, str] = {}
        transitions = []

        for i in range(len(modes)):
            ts, mode = modes[i]
            mode_name = self.FLIGHT_MODES.get(mode, f'Unknown({mode})')
            mode_names[mode] = mode_name

            if i > 0 and modes[i][1] != modes[i - 1][1]:
                transitions.append({
                    'time_s': round((ts - self._t0) / 1e6, 2),
                    'from': self.FLIGHT_MODES.get(modes[i - 1][1], f'Unknown({modes[i-1][1]})'),
                    'to': mode_name,
                })

            if i < len(modes) - 1:
                dt = (modes[i + 1][0] - ts) / 1e6
                mode_times[mode] = mode_times.get(mode, 0) + dt

        result['mode_times_s'] = {mode_names.get(k, str(k)): round(v, 2) for k, v in mode_times.items()}
        result['transitions'] = transitions
        result['transition_count'] = len(transitions)
        result['modes_used'] = list(set(mode_names.values()))

        return result

    # ----------------------------------------------------------
    # 10. Actuator Analysis
    # ----------------------------------------------------------

    def _analyze_actuators(self) -> dict:
        """Actuator saturation and effort analysis."""
        result = {'available': False}

        # Try actuator_motors first, then actuator_outputs
        motor_data = self.ulog.get_topic('actuator_motors')
        output_data = self.ulog.get_topic('actuator_outputs')

        if motor_data:
            result['available'] = True
            result['source'] = 'actuator_motors'
            result['sample_count'] = len(motor_data)

            # Extract motor commands (typically 4-8 motors)
            motor_values: Dict[int, List[float]] = {}
            for d in motor_data:
                for key, val in d.values.items():
                    if key.startswith('control[') and isinstance(val, (int, float)) and _is_valid(val):
                        idx = int(key.split('[')[1].rstrip(']'))
                        if idx not in motor_values:
                            motor_values[idx] = []
                        motor_values[idx].append(val)

            if motor_values:
                motors = {}
                for idx, vals in sorted(motor_values.items()):
                    saturation_count = sum(1 for v in vals if abs(v) >= 0.95)
                    motors[f'motor_{idx}'] = {
                        'mean': round(_mean(vals), 4),
                        'min': round(min(vals), 4),
                        'max': round(max(vals), 4),
                        'std': round(_std(vals), 4),
                        'saturation_pct': round(saturation_count / len(vals) * 100, 1),
                    }
                result['motors'] = motors

                # Overall saturation
                total_sat = sum(m['saturation_pct'] for m in motors.values())
                result['total_saturation_pct'] = round(total_sat / len(motors), 1)

        elif output_data:
            result['available'] = True
            result['source'] = 'actuator_outputs'
            result['sample_count'] = len(output_data)

            # PWM values
            pwm_values: Dict[int, List[float]] = {}
            for d in output_data:
                for key, val in d.values.items():
                    if key.startswith('output[') and isinstance(val, (int, float)):
                        idx = int(key.split('[')[1].rstrip(']'))
                        if idx not in pwm_values:
                            pwm_values[idx] = []
                        pwm_values[idx].append(val)

            if pwm_values:
                motors = {}
                for idx, vals in sorted(pwm_values.items()):
                    if max(vals) > 900:  # Likely PWM values (1000-2000)
                        saturation_count = sum(1 for v in vals if v >= 1950)
                        motors[f'output_{idx}'] = {
                            'mean_pwm': round(_mean(vals), 0),
                            'min_pwm': round(min(vals), 0),
                            'max_pwm': round(max(vals), 0),
                            'saturation_pct': round(saturation_count / len(vals) * 100, 1),
                        }
                    else:
                        motors[f'output_{idx}'] = {
                            'mean': round(_mean(vals), 4),
                            'min': round(min(vals), 4),
                            'max': round(max(vals), 4),
                        }
                result['outputs'] = motors

        if not result['available']:
            result['reason'] = 'No actuator_motors or actuator_outputs data'

        return result

    # ----------------------------------------------------------
    # 11. Controller Configuration
    # ----------------------------------------------------------

    def _analyze_controller_config(self) -> dict:
        """Detect controller type (INDI vs PID) and key parameters."""
        result = {}

        # Check for INDI enable parameter
        indi_enable = self.ulog.get_param('MC_INDI_ENABLE', None)
        if indi_enable is not None:
            result['indi_enabled'] = bool(indi_enable)
            result['controller_type'] = 'INDI' if indi_enable else 'PID'
        else:
            # Try to detect from config_overrides topic
            config_data = self.ulog.get_topic('config_overrides')
            if config_data:
                for d in config_data:
                    if 'MC_INDI_ENABLE' in str(d.values):
                        result['indi_enabled'] = True
                        result['controller_type'] = 'INDI'
                        break
            if 'controller_type' not in result:
                result['controller_type'] = 'unknown'

        # Key control parameters
        param_keys = [
            'MC_ROLLRATE_P', 'MC_ROLLRATE_I', 'MC_ROLLRATE_D',
            'MC_PITCHRATE_P', 'MC_PITCHRATE_I', 'MC_PITCHRATE_D',
            'MC_YAWRATE_P', 'MC_YAWRATE_I', 'MC_YAWRATE_D',
            'MC_INDI_GAIN_P', 'MC_INDI_GAIN_Y', 'MC_INDI_FILTER',
            'MC_ROLL_P', 'MC_PITCH_P', 'MC_YAW_P',
            'MPC_XY_VEL_MAX', 'MPC_Z_VEL_MAX_UP', 'MPC_Z_VEL_MAX_DN',
            'MPC_XY_CRUISE',
            'MC_DZ_X', 'MC_DZ_Y', 'MC_DZ_Z',
        ]

        params_found = {}
        for key in param_keys:
            val = self.ulog.get_param(key, None)
            if val is not None:
                params_found[key] = round(val, 4) if isinstance(val, float) else val

        if params_found:
            result['parameters'] = params_found

        # System info
        sys_name = self.ulog.info.get('sys_name', '')
        if sys_name:
            result['aircraft_model'] = sys_name

        hw_name = self.ulog.info.get('hw_name', '')
        if hw_name:
            result['hardware'] = hw_name

        ver_sw = self.ulog.info.get('ver_sw', '')
        if ver_sw:
            result['software_version'] = ver_sw

        return result

    # ----------------------------------------------------------
    # 12. Performance Rating
    # ----------------------------------------------------------

    def _rate_performance(self, results: dict) -> dict:
        """Compute overall performance rating based on all metrics."""
        score = 100
        issues = []

        # Rate tracking (weight: 25%)
        rt = results.get('rate_tracking', {})
        if rt.get('available'):
            for axis in ['roll', 'pitch', 'yaw']:
                axis_data = rt.get(axis, {})
                abs_err = axis_data.get('abs_error', {})
                rms = abs_err.get('rms_deg_s', 0)
                if rms > 30:
                    score -= 15
                    issues.append(f'Rate tracking {axis} RMS error high: {rms:.1f} deg/s')
                elif rms > 15:
                    score -= 8
                    issues.append(f'Rate tracking {axis} RMS error moderate: {rms:.1f} deg/s')

        # Attitude tracking (weight: 20%)
        at = results.get('attitude_tracking', {})
        if at.get('available'):
            for axis in ['roll', 'pitch']:
                axis_data = at.get(axis, {})
                abs_err = axis_data.get('abs_error', {})
                rms = abs_err.get('rms_deg', 0)
                if rms > 10:
                    score -= 12
                    issues.append(f'Attitude tracking {axis} RMS error high: {rms:.1f} deg')
                elif rms > 5:
                    score -= 6
                    issues.append(f'Attitude tracking {axis} RMS error moderate: {rms:.1f} deg')

        # Position tracking (weight: 20%)
        pt = results.get('position_tracking', {})
        if pt.get('available'):
            err_3d = pt.get('error_3d', {})
            rms = err_3d.get('rms_m', 0)
            if rms > 2.0:
                score -= 15
                issues.append(f'3D position tracking RMS error high: {rms:.2f} m')
            elif rms > 0.5:
                score -= 8
                issues.append(f'3D position tracking RMS error moderate: {rms:.2f} m')

        # Vibration (weight: 15%)
        vib = results.get('vibration', {})
        if vib.get('available'):
            accel_level = vib.get('accel_level', 'good')
            if accel_level == 'high':
                score -= 15
                issues.append('High accelerometer vibration detected')
            elif accel_level == 'acceptable':
                score -= 5
                issues.append('Moderate vibration levels')

            gyro_level = vib.get('gyro_level', 'good')
            if gyro_level == 'high':
                score -= 10
                issues.append('High gyroscope vibration detected')

        # Estimator health (weight: 10%)
        est = results.get('estimator', {})
        if est.get('available'):
            health = est.get('health', 'good')
            if health == 'poor':
                score -= 15
                issues.append('EKF estimator health poor (high test ratios)')
            elif health == 'marginal':
                score -= 7
                issues.append('EKF estimator health marginal')

        # Actuator saturation (weight: 10%)
        act = results.get('actuators', {})
        if act.get('available'):
            sat_pct = act.get('total_saturation_pct', 0)
            if sat_pct > 20:
                score -= 10
                issues.append(f'Actuator saturation high: {sat_pct:.1f}%')
            elif sat_pct > 5:
                score -= 5
                issues.append(f'Actuator saturation moderate: {sat_pct:.1f}%')

        score = max(0, min(100, score))

        if score >= 90:
            grade = 'A'
        elif score >= 80:
            grade = 'B'
        elif score >= 70:
            grade = 'C'
        elif score >= 60:
            grade = 'D'
        else:
            grade = 'F'

        if not issues:
            summary = 'All performance metrics within acceptable range.'
        else:
            summary = f'{len(issues)} issue(s) identified, score {score}/100 (grade {grade}).'

        return {
            'score': score,
            'grade': grade,
            'summary': summary,
            'issues': issues,
        }
