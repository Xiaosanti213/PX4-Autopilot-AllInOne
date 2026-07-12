"""
report_generator.py - Generate structured reports from flight analysis results

Output formats:
  - Markdown: human-readable report with tables and sections
  - JSON: full structured data for programmatic use
  - AI summary: concise one-paragraph summary for LLM consumption
  - Text: plain text summary for console output

Usage:
    from ulog_parser import ULogParser
    from flight_analyzer import FlightPerformanceAnalyzer
    from report_generator import ReportGenerator

    ulog = ULogParser("flight.ulg").parse()
    analyzer = FlightPerformanceAnalyzer(ulog)
    results = analyzer.analyze_with_rating()

    gen = ReportGenerator(results, ulog)
    print(gen.to_markdown())
    print(gen.to_json())
    print(gen.to_ai_summary())
"""

import json
from typing import Dict, Any
from ulog_parser import ULogFile


class ReportGenerator:
    """Generate various report formats from analysis results."""

    def __init__(self, results: dict, ulog: ULogFile):
        self.results = results
        self.ulog = ulog
        self._r = results

    # ----------------------------------------------------------
    # JSON
    # ----------------------------------------------------------

    def to_json(self, indent: int = 2) -> str:
        """Full structured JSON output."""
        return json.dumps(self.results, indent=indent, ensure_ascii=False, default=str)

    # ----------------------------------------------------------
    # Markdown
    # ----------------------------------------------------------

    def to_markdown(self, title: str = "PX4 ULog Flight Performance Report") -> str:
        """Generate a comprehensive Markdown report."""
        r = self._r
        lines = [f"# {title}", ""]

        # Performance Rating
        rating = r.get('performance_rating')
        if rating:
            lines += self._md_rating(rating)

        # Flight Info
        lines += self._md_flight_info()

        # Controller Config
        lines += self._md_controller_config()

        # Attitude
        lines += self._md_attitude()

        # Rate Tracking
        lines += self._md_rate_tracking()

        # Attitude Tracking
        lines += self._md_attitude_tracking()

        # Position Tracking
        lines += self._md_position_tracking()

        # Vibration
        lines += self._md_vibration()

        # Estimator
        lines += self._md_estimator()

        # Battery
        lines += self._md_battery()

        # Flight Modes
        lines += self._md_flight_modes()

        # Actuators
        lines += self._md_actuators()

        # Topics Summary
        lines += self._md_topics()

        return '\n'.join(lines)

    # ----------------------------------------------------------
    # AI Summary
    # ----------------------------------------------------------

    def to_ai_summary(self) -> str:
        """Concise summary suitable for LLM context."""
        r = self._r
        fi = r.get('flight_info', {})
        parts = []

        # Basic info
        dur = fi.get('duration_s', 0)
        armed = fi.get('armed', False)
        armed_time = fi.get('armed_time_s', 0)
        parts.append(f"Flight: {dur:.0f}s duration, {'armed' if armed else 'not armed'}"
                     f"{f' for {armed_time:.0f}s' if armed else ''}, "
                     f"{fi.get('topic_count', 0)} topics logged.")

        # Controller
        cc = r.get('controller_config', {})
        if cc:
            parts.append(f"Controller: {cc.get('controller_type', 'unknown')}")

        # Performance rating
        rating = r.get('performance_rating')
        if rating:
            parts.append(f"Rating: {rating['score']}/100 (grade {rating['grade']}). "
                         f"{rating['summary']}")
            if rating['issues']:
                parts.append("Issues: " + "; ".join(rating['issues']))

        # Attitude
        att = r.get('attitude', {})
        if att and 'euler_deg' in att:
            e = att['euler_deg']
            parts.append(
                f"Attitude: Roll {e['roll']['mean']:.1f}+/-{e['roll']['std']:.1f}deg, "
                f"Pitch {e['pitch']['mean']:.1f}+/-{e['pitch']['std']:.1f}deg."
            )

        # Rate tracking
        rt = r.get('rate_tracking', {})
        if rt.get('available'):
            errs = []
            for axis in ['roll', 'pitch', 'yaw']:
                ad = rt.get(axis, {})
                ae = ad.get('abs_error', {})
                if ae:
                    errs.append(f"{axis} RMS={ae.get('rms_deg_s', 0):.2f}deg/s")
            if errs:
                parts.append("Rate tracking: " + ", ".join(errs))

        # Position tracking
        pt = r.get('position_tracking', {})
        if pt.get('available'):
            e3d = pt.get('error_3d', {})
            if e3d:
                parts.append(f"Position 3D error: RMS={e3d.get('rms_m', 0):.3f}m, "
                             f"max={e3d.get('max_m', 0):.3f}m.")

        # Vibration
        vib = r.get('vibration', {})
        if vib.get('available'):
            parts.append(f"Vibration: gyro={vib.get('gyro_level', '?')}, "
                         f"accel={vib.get('accel_level', '?')}.")

        # Battery
        bat = r.get('battery', {})
        if bat.get('available'):
            v = bat.get('voltage_v', {})
            if v:
                parts.append(f"Battery: {v.get('start', 0):.1f}V -> {v.get('end', 0):.1f}V"
                             f" (drop {bat.get('voltage_drop_v', 0):.1f}V), "
                             f"consumed {bat.get('consumed_mah', 0):.0f}mAh.")

        return ' '.join(parts)

    # ----------------------------------------------------------
    # Plain text (for console)
    # ----------------------------------------------------------

    def to_text(self) -> str:
        """Plain text summary for console output."""
        r = self._r
        fi = r.get('flight_info', {})
        lines = []

        lines.append("=" * 60)
        lines.append("  PX4 ULog Flight Performance Analysis")
        lines.append("=" * 60)
        lines.append("")

        # Rating
        rating = r.get('performance_rating')
        if rating:
            lines.append(f"  Overall Score: {rating['score']}/100  (Grade: {rating['grade']})")
            lines.append(f"  {rating['summary']}")
            if rating['issues']:
                lines.append("")
                for issue in rating['issues']:
                    lines.append(f"  [!] {issue}")
            lines.append("")

        # Flight info
        lines.append("[Flight Info]")
        lines.append(f"  Duration:      {fi.get('duration_s', 0):.1f} s")
        lines.append(f"  Armed:         {'Yes' if fi.get('armed') else 'No'}"
                     f" ({fi.get('armed_time_s', 0):.1f}s)")
        lines.append(f"  Topics logged: {fi.get('topic_count', 0)}")
        lines.append("")

        # Controller
        cc = r.get('controller_config', {})
        if cc:
            lines.append("[Controller]")
            lines.append(f"  Type:          {cc.get('controller_type', 'unknown')}")
            if cc.get('aircraft_model'):
                lines.append(f"  Model:         {cc['aircraft_model']}")
            lines.append("")

        # Attitude
        att = r.get('attitude', {})
        if att and 'euler_deg' in att:
            lines.append("[Attitude]")
            for axis in ['roll', 'pitch', 'yaw']:
                s = att['euler_deg'][axis]
                lines.append(f"  {axis.capitalize():6s}: mean={s['mean']:.1f}  "
                             f"std={s['std']:.2f}  range=[{s['min']:.1f}, {s['max']:.1f}] deg")
            lines.append("")

        # Rate tracking
        rt = r.get('rate_tracking', {})
        if rt.get('available'):
            lines.append("[Rate Tracking]")
            for axis in ['roll', 'pitch', 'yaw']:
                ad = rt.get(axis, {})
                ae = ad.get('abs_error', {})
                if ae:
                    lines.append(f"  {axis.capitalize():6s}: RMS={ae.get('rms_deg_s', 0):.3f}  "
                                 f"max={ae.get('max_deg_s', 0):.3f} deg/s")
            lines.append("")

        # Attitude tracking
        at = r.get('attitude_tracking', {})
        if at.get('available'):
            lines.append("[Attitude Tracking]")
            for axis in ['roll', 'pitch', 'yaw']:
                ad = at.get(axis, {})
                ae = ad.get('abs_error', {})
                if ae:
                    lines.append(f"  {axis.capitalize():6s}: RMS={ae.get('rms_deg', 0):.3f}  "
                                 f"max={ae.get('max_deg', 0):.3f} deg")
            lines.append("")

        # Position tracking
        pt = r.get('position_tracking', {})
        if pt.get('available'):
            lines.append("[Position Tracking]")
            e3d = pt.get('error_3d', {})
            if e3d:
                lines.append(f"  3D Error:      RMS={e3d.get('rms_m', 0):.3f}m  "
                             f"max={e3d.get('max_m', 0):.3f}m")
            alt = pt.get('altitude', {})
            if alt:
                lines.append(f"  Altitude:      {alt.get('min_m', 0):.1f} ~ {alt.get('max_m', 0):.1f} m")
            lines.append("")

        # Estimator
        est = r.get('estimator', {})
        if est.get('available'):
            lines.append("[Estimator (EKF)]")
            lines.append(f"  Health:        {est.get('health', '?')}")
            for field, label in [('pos_test_ratio', 'Position'), ('vel_test_ratio', 'Velocity'),
                                 ('hgt_test_ratio', 'Height'), ('hdg_test_ratio', 'Heading')]:
                s = est.get(field, {})
                if s:
                    lines.append(f"  {label:15s} ratio: mean={s.get('mean', 0):.3f}  max={s.get('max', 0):.3f}")
            lines.append("")

        # Flight modes
        fm = r.get('flight_modes', {})
        if fm.get('available'):
            lines.append("[Flight Modes]")
            lines.append(f"  Transitions:   {fm.get('transition_count', 0)}")
            lines.append(f"  Modes:         {', '.join(fm.get('modes_used', []))}")
            lines.append("")

        # Actuators
        act = r.get('actuators', {})
        if act.get('available'):
            lines.append("[Actuators]")
            lines.append(f"  Source:        {act.get('source', '?')}")
            if act.get('total_saturation_pct') is not None:
                lines.append(f"  Saturation:    {act.get('total_saturation_pct', 0):.1f}%")
            lines.append("")

        # Vibration
        vib = r.get('vibration', {})
        if vib.get('available'):
            lines.append("[Vibration]")
            lines.append(f"  Gyro level:    {vib.get('gyro_level', '?')}")
            lines.append(f"  Accel level:   {vib.get('accel_level', '?')}")
            lines.append("")

        # Battery
        bat = r.get('battery', {})
        if bat.get('available'):
            lines.append("[Battery]")
            v = bat.get('voltage_v', {})
            if v:
                lines.append(f"  Voltage:       {v.get('start', 0):.1f}V -> {v.get('end', 0):.1f}V"
                             f" (drop {bat.get('voltage_drop_v', 0):.1f}V)")
            c = bat.get('current_a', {})
            if c:
                lines.append(f"  Current:       mean={c.get('mean', 0):.1f}A  max={c.get('max', 0):.1f}A")
            if bat.get('consumed_mah'):
                lines.append(f"  Consumed:      {bat['consumed_mah']:.0f} mAh")
            lines.append("")

        lines.append("=" * 60)
        return '\n'.join(lines)

    # ==========================================================
    # Markdown section builders
    # ==========================================================

    def _md_rating(self, rating: dict) -> list:
        r = rating
        lines = [
            "## Performance Rating",
            "",
            f"| Score | Grade | Summary |",
            f"|-------|-------|---------|",
            f"| **{r['score']}/100** | **{r['grade']}** | {r['summary']} |",
            "",
        ]
        if r['issues']:
            lines.append("### Issues")
            lines.append("")
            for issue in r['issues']:
                lines.append(f"- {issue}")
            lines.append("")
        return lines

    def _md_flight_info(self) -> list:
        fi = self._r.get('flight_info', {})
        lines = [
            "## Flight Overview",
            "",
            f"- **Duration**: {fi.get('duration_s', 0):.1f} s",
            f"- **Armed**: {'Yes' if fi.get('armed') else 'No'}"
            f" ({fi.get('armed_time_s', 0):.1f} s)",
            f"- **Topics logged**: {fi.get('topic_count', 0)}",
            "",
        ]
        return lines

    def _md_controller_config(self) -> list:
        cc = self._r.get('controller_config', {})
        if not cc:
            return []
        lines = ["## Controller Configuration", ""]

        ctrl_type = cc.get('controller_type', 'unknown')
        indi = cc.get('indi_enabled')
        if indi is not None:
            lines.append(f"- **Controller type**: {ctrl_type} (INDI {'enabled' if indi else 'disabled'})")
        else:
            lines.append(f"- **Controller type**: {ctrl_type}")

        if cc.get('aircraft_model'):
            lines.append(f"- **Aircraft model**: {cc['aircraft_model']}")
        if cc.get('hardware'):
            lines.append(f"- **Hardware**: {cc['hardware']}")
        if cc.get('software_version'):
            lines.append(f"- **Software version**: {cc['software_version']}")

        params = cc.get('parameters', {})
        if params:
            lines.append("")
            lines.append("### Key Parameters")
            lines.append("")
            lines.append("| Parameter | Value |")
            lines.append("|-----------|-------|")
            for k, v in sorted(params.items()):
                lines.append(f"| `{k}` | {v} |")

        lines.append("")
        return lines

    def _md_attitude(self) -> list:
        att = self._r.get('attitude', {})
        if not att or 'euler_deg' not in att:
            return []

        e = att['euler_deg']
        lines = [
            "## Attitude Statistics",
            "",
            "| Axis | Mean | Std | Min | Max |",
            "|------|------|-----|-----|-----|",
        ]
        for axis in ['roll', 'pitch', 'yaw']:
            s = e[axis]
            lines.append(f"| {axis.capitalize()} | {s['mean']:.1f} deg | {s['std']:.2f} deg | "
                         f"{s['min']:.1f} deg | {s['max']:.1f} deg |")
        lines.append("")

        # Angular rates
        ar = att.get('angular_rate_rad_s', {})
        if ar:
            lines.append("### Angular Rates")
            lines.append("")
            lines.append("| Axis | Mean (rad/s) | RMS (rad/s) | Max (rad/s) |")
            lines.append("|------|-------------|-------------|-------------|")
            for axis in ['roll', 'pitch', 'yaw']:
                s = ar.get(axis, {})
                lines.append(f"| {axis.capitalize()} | {s.get('mean', 0):.4f} | "
                             f"{s.get('rms', 0):.4f} | {s.get('max', 0):.4f} |")
            lines.append("")

        return lines

    def _md_rate_tracking(self) -> list:
        rt = self._r.get('rate_tracking', {})
        if not rt.get('available'):
            return []

        lines = [
            "## Rate Tracking Performance",
            "",
            "| Axis | RMS Error (deg/s) | Mean Error (deg/s) | Max Error (deg/s) |",
            "|------|-------------------|--------------------|--------------------|",
        ]
        for axis in ['roll', 'pitch', 'yaw']:
            ad = rt.get(axis, {})
            ae = ad.get('abs_error', {})
            if ae:
                lines.append(f"| {axis.capitalize()} | {ae.get('rms_deg_s', 0):.3f} | "
                             f"{ae.get('mean_deg_s', 0):.3f} | {ae.get('max_deg_s', 0):.3f} |")
            else:
                lines.append(f"| {axis.capitalize()} | N/A | N/A | N/A |")
        lines.append("")
        return lines

    def _md_attitude_tracking(self) -> list:
        at = self._r.get('attitude_tracking', {})
        if not at.get('available'):
            return []

        lines = [
            "## Attitude Tracking Performance",
            "",
            "| Axis | RMS Error (deg) | Mean Error (deg) | Max Error (deg) |",
            "|------|-----------------|------------------|------------------|",
        ]
        for axis in ['roll', 'pitch', 'yaw']:
            ad = at.get(axis, {})
            ae = ad.get('abs_error', {})
            if ae:
                lines.append(f"| {axis.capitalize()} | {ae.get('rms_deg', 0):.3f} | "
                             f"{ae.get('mean_deg', 0):.3f} | {ae.get('max_deg', 0):.3f} |")
            else:
                lines.append(f"| {axis.capitalize()} | N/A | N/A | N/A |")
        lines.append("")
        return lines

    def _md_position_tracking(self) -> list:
        pt = self._r.get('position_tracking', {})
        if not pt.get('available'):
            return []

        lines = ["## Position Tracking Performance", ""]

        # 3D error
        e3d = pt.get('error_3d', {})
        if e3d:
            lines.append(f"- **3D RMS Error**: {e3d.get('rms_m', 0):.3f} m")
            lines.append(f"- **3D Max Error**: {e3d.get('max_m', 0):.3f} m")
            lines.append("")

        # Per-axis
        lines.append("| Axis | RMS Error (m) | Max Error (m) |")
        lines.append("|------|--------------|--------------|")
        for axis in ['x', 'y', 'z']:
            ad = pt.get(axis, {})
            ae = ad.get('abs_error', {})
            if ae:
                lines.append(f"| {axis.upper()} | {ae.get('rms_m', 0):.4f} | {ae.get('max_m', 0):.4f} |")
            else:
                lines.append(f"| {axis.upper()} | N/A | N/A |")
        lines.append("")

        # Altitude
        alt = pt.get('altitude', {})
        if alt:
            lines.append(f"- **Altitude range**: {alt.get('min_m', 0):.1f} ~ {alt.get('max_m', 0):.1f} m")
            lines.append("")

        return lines

    def _md_vibration(self) -> list:
        vib = self._r.get('vibration', {})
        if not vib.get('available'):
            return []

        lines = ["## Vibration Analysis", ""]

        lines.append(f"- **Gyroscope level**: {vib.get('gyro_level', 'unknown')}")
        lines.append(f"- **Accelerometer level**: {vib.get('accel_level', 'unknown')}")
        lines.append("")

        # Gyro details
        gyro = vib.get('gyro', {})
        if gyro:
            lines.append("### Gyroscope Vibration (rad/s)")
            lines.append("")
            lines.append("| Axis | Peak-to-Peak | RMS | Std |")
            lines.append("|------|-------------|-----|-----|")
            for axis in ['x', 'y', 'z']:
                s = gyro.get(axis, {})
                lines.append(f"| {axis.upper()} | {s.get('peak_to_peak_rad_s', 0):.4f} | "
                             f"{s.get('rms_rad_s', 0):.4f} | {s.get('std_rad_s', 0):.4f} |")
            lines.append("")

        # Accel details
        accel = vib.get('accel', {})
        if accel:
            lines.append("### Accelerometer Vibration (m/s^2)")
            lines.append("")
            lines.append("| Axis | Peak-to-Peak | RMS | Std |")
            lines.append("|------|-------------|-----|-----|")
            for axis in ['x', 'y', 'z']:
                s = accel.get(axis, {})
                lines.append(f"| {axis.upper()} | {s.get('peak_to_peak_m_s2', 0):.2f} | "
                             f"{s.get('rms_m_s2', 0):.2f} | {s.get('std_m_s2', 0):.2f} |")
            lines.append("")

        return lines

    def _md_estimator(self) -> list:
        est = self._r.get('estimator', {})
        if not est.get('available'):
            return []

        lines = ["## Estimator (EKF) Performance", ""]
        lines.append(f"- **Health**: {est.get('health', 'unknown')}")
        lines.append("")

        # Test ratios
        ratio_fields = [
            ('pos_test_ratio', 'Position'),
            ('vel_test_ratio', 'Velocity'),
            ('mag_test_ratio', 'Magnetometer'),
            ('hgt_test_ratio', 'Height'),
        ]
        has_ratios = any(est.get(f) for f, _ in ratio_fields)
        if has_ratios:
            lines.append("| Source | Mean | Max | P95 |")
            lines.append("|--------|------|-----|-----|")
            for field, label in ratio_fields:
                s = est.get(field, {})
                if s:
                    lines.append(f"| {label} | {s.get('mean', 0):.4f} | "
                                 f"{s.get('max', 0):.4f} | {s.get('p95', 0):.4f} |")
            lines.append("")

        # Innovations
        innov = est.get('innovations', {})
        if innov:
            lines.append("### Innovations")
            lines.append("")
            lines.append("| Source | Mean | Std | Min | Max |")
            lines.append("|--------|------|-----|-----|-----|")
            for label, s in innov.items():
                lines.append(f"| {label} | {s.get('mean', 0):.4f} | {s.get('std', 0):.4f} | "
                             f"{s.get('min', 0):.4f} | {s.get('max', 0):.4f} |")
            lines.append("")

        return lines

    def _md_battery(self) -> list:
        bat = self._r.get('battery', {})
        if not bat.get('available'):
            return []

        lines = ["## Battery & Power", ""]
        v = bat.get('voltage_v', {})
        if v:
            lines.append(f"- **Voltage**: {v.get('start', 0):.1f}V -> {v.get('end', 0):.1f}V "
                         f"(drop {bat.get('voltage_drop_v', 0):.1f}V)")
            lines.append(f"- **Voltage range**: {v.get('min', 0):.1f}V ~ {v.get('max', 0):.1f}V")

        c = bat.get('current_a', {})
        if c:
            lines.append(f"- **Current**: mean {c.get('mean', 0):.1f}A, max {c.get('max', 0):.1f}A")

        if bat.get('consumed_mah'):
            lines.append(f"- **Energy consumed**: {bat['consumed_mah']:.0f} mAh")

        p = bat.get('power_w', {})
        if p:
            lines.append(f"- **Power**: mean {p.get('mean', 0):.1f}W, max {p.get('max', 0):.1f}W")

        rem = bat.get('remaining_pct', {})
        if rem:
            lines.append(f"- **Remaining**: {rem.get('start', 0):.1f}% -> {rem.get('end', 0):.1f}%")

        lines.append("")
        return lines

    def _md_flight_modes(self) -> list:
        fm = self._r.get('flight_modes', {})
        if not fm.get('available'):
            return []

        lines = ["## Flight Mode Analysis", ""]
        lines.append(f"- **Transitions**: {fm.get('transition_count', 0)}")
        lines.append(f"- **Modes used**: {', '.join(fm.get('modes_used', []))}")
        lines.append("")

        mode_times = fm.get('mode_times_s', {})
        if mode_times:
            lines.append("### Time per Mode")
            lines.append("")
            lines.append("| Mode | Duration (s) |")
            lines.append("|------|-------------|")
            for mode, t in sorted(mode_times.items(), key=lambda x: -x[1]):
                lines.append(f"| {mode} | {t:.1f} |")
            lines.append("")

        transitions = fm.get('transitions', [])
        if transitions:
            lines.append("### Mode Transitions")
            lines.append("")
            lines.append("| Time (s) | From | To |")
            lines.append("|----------|------|-----|")
            for t in transitions:
                lines.append(f"| {t['time_s']:.1f} | {t['from']} | {t['to']} |")
            lines.append("")

        return lines

    def _md_actuators(self) -> list:
        act = self._r.get('actuators', {})
        if not act.get('available'):
            return []

        lines = ["## Actuator Analysis", ""]
        lines.append(f"- **Source**: {act.get('source', 'unknown')}")
        if act.get('total_saturation_pct') is not None:
            lines.append(f"- **Total saturation**: {act.get('total_saturation_pct', 0):.1f}%")
        lines.append("")

        motors = act.get('motors', act.get('outputs', {}))
        if motors:
            if act.get('source') == 'actuator_motors':
                lines.append("| Motor | Mean | Min | Max | Saturation % |")
                lines.append("|-------|------|-----|-----|-------------|")
                for name, s in motors.items():
                    lines.append(f"| {name} | {s.get('mean', 0):.4f} | {s.get('min', 0):.4f} | "
                                 f"{s.get('max', 0):.4f} | {s.get('saturation_pct', 0):.1f}% |")
            else:
                lines.append("| Output | Mean PWM | Min PWM | Max PWM | Saturation % |")
                lines.append("|--------|----------|---------|---------|-------------|")
                for name, s in motors.items():
                    lines.append(f"| {name} | {s.get('mean_pwm', 0):.0f} | {s.get('min_pwm', 0):.0f} | "
                                 f"{s.get('max_pwm', 0):.0f} | {s.get('saturation_pct', 0):.1f}% |")
            lines.append("")

        return lines

    def _md_topics(self) -> list:
        fi = self._r.get('flight_info', {})
        topic_counts = fi.get('topic_counts', {})
        if not topic_counts:
            return []

        lines = ["## Logged Topics", ""]
        lines.append("| Topic | Samples |")
        lines.append("|-------|---------|")
        for topic in sorted(topic_counts.keys()):
            lines.append(f"| `{topic}` | {topic_counts[topic]} |")
        lines.append("")

        return lines
