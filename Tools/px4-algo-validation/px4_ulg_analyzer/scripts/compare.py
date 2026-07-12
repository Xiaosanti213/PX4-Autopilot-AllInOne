#!/usr/bin/env python3
"""
compare.py - Side-by-side PX4 ULog flight performance comparison

The primary tool for comparing two flight logs — e.g., INDI vs PID,
different tuning parameters, or hardware variants.

Usage:
    # Basic comparison
    python compare.py indi.ulg pid.ulg

    # With custom labels
    python compare.py indi.ulg pid.ulg --labels "INDI Controller" "PID Controller"

    # JSON output
    python compare.py indi.ulg pid.ulg --json

    # Less output noise (best in CI/scripts)
    python compare.py indi.ulg pid.ulg -q

    # JSON to file
    python compare.py indi.ulg pid.ulg --json -o comparison.json

Python API:
    from compare import ULogComparer
    comp = ULogComparer("indi.ulg", "pid.ulg")
    comp.run()
    print(comp.to_text())
    data = comp.to_dict()  # programmatic access
"""

import argparse
import sys
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field

sys.path.insert(0, str(Path(__file__).parent))

from ulog_parser import ULogParser
from flight_analyzer import FlightPerformanceAnalyzer


# ============================================================
# Data classes
# ============================================================

@dataclass
class MetricComparison:
    """Single metric comparison result."""
    name: str                                        # eg. "Rate Tracking RMS — Roll"
    unit: str                                        # eg. "deg/s"
    value_a: Optional[float] = None
    value_b: Optional[float] = None
    label_a: str = ""                                # eg. "4.23"
    label_b: str = ""                                # eg. "5.78"
    lower_is_better: bool = True
    winner: Optional[str] = None                     # "A", "B", or None (tie/unavailable)
    delta_pct: Optional[float] = None                # positive = A is better (when lower_is_better)


@dataclass
class SectionComparison:
    """A named group of related metrics."""
    name: str                                        # eg. "Rate Tracking"
    metrics: List[MetricComparison] = field(default_factory=list)


# ============================================================
# ULog Comparer
# ============================================================

class ULogComparer:
    """Load, analyze, and compare two PX4 ULog files."""

    AUTO_LABEL_PATTERNS = {
        'indi': 'INDI',
        'pid': 'PID',
        'indi_enable': 'INDI',
        'pid_enabled': 'PID',
        'tuned': 'Tuned',
        'default': 'Default',
        'baseline': 'Baseline',
        'test': 'Test',
    }

    def __init__(self, file_a: str, file_b: str,
                 label_a: str = "", label_b: str = ""):
        self.file_a = Path(file_a)
        self.file_b = Path(file_b)
        self.label_a = label_a or "A"
        self.label_b = label_b or "B"
        self.ulog_a: Any = None
        self.ulog_b: Any = None
        self.results_a: dict = {}
        self.results_b: dict = {}
        self.sections: List[SectionComparison] = []
        self._parse_times: Dict[str, float] = {}

    def run(self) -> "ULogComparer":
        """Parse both ULogs and run all analyses. Returns self for chaining."""
        # Parse A
        t0 = time.time()
        self.ulog_a = ULogParser(str(self.file_a)).parse()
        self._parse_times['a'] = time.time() - t0

        # Parse B
        t0 = time.time()
        self.ulog_b = ULogParser(str(self.file_b)).parse()
        self._parse_times['b'] = time.time() - t0

        # Auto-detect labels from controller type
        if self.label_a == "A":
            analyzer_a = FlightPerformanceAnalyzer(self.ulog_a)
            ctrl_cfg_a = analyzer_a._analyze_controller_config()
            ctrl_type_a = ctrl_cfg_a.get('controller_type', '')
            if ctrl_type_a and ctrl_type_a not in ('unknown',):
                self.label_a = ctrl_type_a
            else:
                # Try file name hints
                stem = self.file_a.stem.lower()
                for pattern, label in self.AUTO_LABEL_PATTERNS.items():
                    if pattern in stem:
                        self.label_a = label
                        break
                if self.label_a == "A":
                    self.label_a = self.file_a.stem[:20]

        if self.label_b == "B":
            analyzer_b = FlightPerformanceAnalyzer(self.ulog_b)
            ctrl_cfg_b = analyzer_b._analyze_controller_config()
            ctrl_type_b = ctrl_cfg_b.get('controller_type', '')
            if ctrl_type_b and ctrl_type_b not in ('unknown',):
                self.label_b = ctrl_type_b
            else:
                stem = self.file_b.stem.lower()
                for pattern, label in self.AUTO_LABEL_PATTERNS.items():
                    if pattern in stem:
                        self.label_b = label
                        break
                if self.label_b == "B":
                    self.label_b = self.file_b.stem[:20]

        # Analyze both
        t0 = time.time()
        self.results_a = FlightPerformanceAnalyzer(self.ulog_a).analyze_with_rating()
        self._parse_times['analysis_a'] = time.time() - t0

        t0 = time.time()
        self.results_b = FlightPerformanceAnalyzer(self.ulog_b).analyze_with_rating()
        self._parse_times['analysis_b'] = time.time() - t0

        # Build comparison sections
        self._build_comparison()
        return self

    # ---- Comparison Building ----

    def _build_comparison(self):
        self.sections = [
            self._cmp_rate_tracking(),
            self._cmp_attitude_tracking(),
            self._cmp_position_tracking(),
            self._cmp_vibration(),
            self._cmp_estimator(),
            self._cmp_actuators(),
            self._cmp_battery(),
            self._cmp_overall(),
        ]

    def _get_rt_axis(self, results: dict, axis: str) -> dict:
        rt = results.get('rate_tracking', {})
        if not rt.get('available'):
            return {}
        return rt.get(axis, {}).get('abs_error', {})

    def _cmp_rate_tracking(self) -> SectionComparison:
        sec = SectionComparison(name="Rate Tracking")
        for axis in ['roll', 'pitch', 'yaw']:
            ae_a = self._get_rt_axis(self.results_a, axis)
            ae_b = self._get_rt_axis(self.results_b, axis)
            sec.metrics.append(self._mk_metric(
                f"  {axis}", "deg/s",
                ae_a.get('rms_deg_s'), ae_b.get('rms_deg_s'),
                fmt=".2f"
            ))
        return sec

    def _get_at_axis(self, results: dict, axis: str) -> dict:
        at = results.get('attitude_tracking', {})
        if not at.get('available'):
            return {}
        return at.get(axis, {}).get('abs_error', {})

    def _cmp_attitude_tracking(self) -> SectionComparison:
        sec = SectionComparison(name="Attitude Tracking")
        for axis in ['roll', 'pitch', 'yaw']:
            ae_a = self._get_at_axis(self.results_a, axis)
            ae_b = self._get_at_axis(self.results_b, axis)
            sec.metrics.append(self._mk_metric(
                f"  {axis}", "deg",
                ae_a.get('rms_deg'), ae_b.get('rms_deg'),
                fmt=".1f"
            ))
        return sec

    def _cmp_position_tracking(self) -> SectionComparison:
        sec = SectionComparison(name="Position Tracking")
        pt_a = self.results_a.get('position_tracking', {})
        pt_b = self.results_b.get('position_tracking', {})

        err_a = pt_a.get('error_3d', {})
        err_b = pt_b.get('error_3d', {})
        sec.metrics.append(self._mk_metric(
            "  3D RMS", "m",
            err_a.get('rms_m'), err_b.get('rms_m'),
            fmt=".3f"
        ))

        # Per-axis
        for axis in ['x', 'y', 'z']:
            ax_a = pt_a.get(axis, {}).get('abs_error', {})
            ax_b = pt_b.get(axis, {}).get('abs_error', {})
            sec.metrics.append(self._mk_metric(
                f"  {axis} RMS", "m",
                ax_a.get('rms_m'), ax_b.get('rms_m'),
                fmt=".3f"
            ))
        return sec

    def _cmp_vibration(self) -> SectionComparison:
        sec = SectionComparison(name="Vibration")
        vib_a = self.results_a.get('vibration', {})
        vib_b = self.results_b.get('vibration', {})

        # Gyro per-axis peak-to-peak
        gyro_a = vib_a.get('gyro', {})
        gyro_b = vib_b.get('gyro', {})
        for axis in ['x', 'y', 'z']:
            sec.metrics.append(self._mk_metric(
                f"  Gyro {axis} P2P", "rad/s",
                gyro_a.get(axis, {}).get('peak_to_peak_rad_s'),
                gyro_b.get(axis, {}).get('peak_to_peak_rad_s'),
                fmt=".3f"
            ))

        # Accel per-axis peak-to-peak
        accel_a = vib_a.get('accel', {})
        accel_b = vib_b.get('accel', {})
        for axis in ['x', 'y', 'z']:
            sec.metrics.append(self._mk_metric(
                f"  Accel {axis} P2P", "m/s²",
                accel_a.get(axis, {}).get('peak_to_peak_m_s2'),
                accel_b.get(axis, {}).get('peak_to_peak_m_s2'),
                fmt=".1f"
            ))

        # Overall levels
        sec.metrics.append(self._mk_metric(
            "  Gyro level", "",
            gyro_level_a := str(vib_a.get('gyro_level', '?')),
            gyro_level_b := str(vib_b.get('gyro_level', '?')),
            is_string=True
        ))
        sec.metrics.append(self._mk_metric(
            "  Accel level", "",
            str(vib_a.get('accel_level', '?')),
            str(vib_b.get('accel_level', '?')),
            is_string=True
        ))
        return sec

    def _cmp_estimator(self) -> SectionComparison:
        sec = SectionComparison(name="EKF Estimator")
        est_a = self.results_a.get('estimator', {})
        est_b = self.results_b.get('estimator', {})

        for field in ['pos_test_ratio', 'vel_test_ratio', 'hgt_test_ratio', 'hdg_test_ratio']:
            vals_a = est_a.get(field, {})
            vals_b = est_b.get(field, {})
            sec.metrics.append(self._mk_metric(
                f"  {field}", "",
                vals_a.get('max'), vals_b.get('max'),
                fmt=".2f"
            ))

        sec.metrics.append(self._mk_metric(
            "  Health", "",
            str(est_a.get('health', '?')),
            str(est_b.get('health', '?')),
            is_string=True
        ))
        return sec

    def _cmp_actuators(self) -> SectionComparison:
        sec = SectionComparison(name="Actuators")
        act_a = self.results_a.get('actuators', {})
        act_b = self.results_b.get('actuators', {})

        sec.metrics.append(self._mk_metric(
            "  Saturation", "%",
            act_a.get('total_saturation_pct'),
            act_b.get('total_saturation_pct'),
            fmt=".1f"
        ))

        # Per-motor mean effort
        motors_a = act_a.get('motors') or act_a.get('outputs') or {}
        motors_b = act_b.get('motors') or act_b.get('outputs') or {}
        motor_keys = sorted(set(motors_a.keys()) | set(motors_b.keys()))
        for key in list(motor_keys)[:6]:  # max 6 motors
            short_name = key.replace('motor_', 'M').replace('output_', 'O')
            info_a = motors_a.get(key, {})
            info_b = motors_b.get(key, {})
            val_a = info_a.get('mean') or info_a.get('mean_pwm')
            val_b = info_b.get('mean') or info_b.get('mean_pwm')
            sec.metrics.append(self._mk_metric(
                f"  {short_name} mean", "",
                val_a, val_b,
                fmt=".3f", lower_is_better=False  # effort is context-dependent
            ))
        return sec

    def _cmp_battery(self) -> SectionComparison:
        sec = SectionComparison(name="Battery")
        bat_a = self.results_a.get('battery', {})
        bat_b = self.results_b.get('battery', {})

        sec.metrics.append(self._mk_metric(
            "  Consumed", "mAh",
            bat_a.get('consumed_mah'), bat_b.get('consumed_mah'),
            fmt=".0f"
        ))
        sec.metrics.append(self._mk_metric(
            "  Voltage drop", "V",
            bat_a.get('voltage_drop_v'), bat_b.get('voltage_drop_v'),
            fmt=".2f"
        ))
        sec.metrics.append(self._mk_metric(
            "  Mean current", "A",
            bat_a.get('current_a', {}).get('mean'),
            bat_b.get('current_a', {}).get('mean'),
            fmt=".1f"
        ))
        sec.metrics.append(self._mk_metric(
            "  Mean power", "W",
            bat_a.get('power_w', {}).get('mean'),
            bat_b.get('power_w', {}).get('mean'),
            fmt=".1f"
        ))
        return sec

    def _cmp_overall(self) -> SectionComparison:
        sec = SectionComparison(name="Overall")
        rating_a = self.results_a.get('performance_rating', {})
        rating_b = self.results_b.get('performance_rating', {})

        sec.metrics.append(self._mk_metric(
            "  Score", "",
            rating_a.get('score'), rating_b.get('score'),
            fmt=".0f", lower_is_better=False  # higher score is better
        ))
        sec.metrics.append(self._mk_metric(
            "  Grade", "",
            str(rating_a.get('grade', '?')),
            str(rating_b.get('grade', '?')),
            is_string=True
        ))
        return sec

    def _mk_metric(self, name: str, unit: str,
                   val_a: Any, val_b: Any,
                   fmt: str = ".2f",
                   lower_is_better: bool = True,
                   is_string: bool = False) -> MetricComparison:
        """Create a MetricComparison with winner and delta auto-computed."""
        m = MetricComparison(
            name=name,
            unit=unit,
            lower_is_better=lower_is_better,
        )

        # Format values
        if is_string:
            m.label_a = str(val_a)
            m.label_b = str(val_b)
            m.value_a = None
            m.value_b = None
            # String comparison: see if they differ
            if val_a and val_b and val_a != val_b:
                # Can't determine winner for arbitrary strings
                m.winner = None
            return m

        # Numeric comparison
        if val_a is not None and isinstance(val_a, (int, float)):
            m.value_a = float(val_a)
            m.label_a = f"{val_a:{fmt}}"
        else:
            m.label_a = "N/A"

        if val_b is not None and isinstance(val_b, (int, float)):
            m.value_b = float(val_b)
            m.label_b = f"{val_b:{fmt}}"
        else:
            m.label_b = "N/A"

        # Compute winner and delta
        if m.value_a is not None and m.value_b is not None:
            # Only compute delta when both values are meaningful
            if abs(m.value_b) > 1e-9:
                m.delta_pct = (m.value_a - m.value_b) / abs(m.value_b) * 100
            elif abs(m.value_a) > 1e-9:
                m.delta_pct = float('inf')  # huge improvement, don't show number
            # else both near zero — no delta

            if abs(m.value_a - m.value_b) < 1e-6:
                m.winner = None  # tie
            elif lower_is_better:
                m.winner = "A" if m.value_a < m.value_b else "B"
            else:
                m.winner = "A" if m.value_a > m.value_b else "B"

        return m

    # ---- Output ----

    def to_text(self) -> str:
        """Generate a clean comparison table for console output."""
        label_a_short = self.label_a[:12]
        label_b_short = self.label_b[:12]
        duration_a = self.results_a.get('flight_info', {}).get('duration_s', 0)
        duration_b = self.results_b.get('flight_info', {}).get('duration_s', 0)
        topics_a = self.results_a.get('flight_info', {}).get('topic_count', 0)
        topics_b = self.results_b.get('flight_info', {}).get('topic_count', 0)

        lines = []
        lines.append("═" * 72)
        lines.append("  PX4 Flight Performance Comparison")
        lines.append("═" * 72)
        lines.append(f"  {label_a_short:>12s}: {self.file_a.name}  ({duration_a:.1f}s, {topics_a} topics)")
        lines.append(f"  {label_b_short:>12s}: {self.file_b.name}  ({duration_b:.1f}s, {topics_b} topics)")
        lines.append("─" * 72)

        # Metrics table
        hdr = f"  {'Metric':<26s} {'':>5s} {label_a_short:>8s}  {label_b_short:>8s}  {'Δ%':>6s}"
        lines.append(hdr)
        lines.append("─" * 72)

        total_metrics = 0
        wins_a = 0
        wins_b = 0

        for section in self.sections:
            # Section header
            lines.append(f"  {section.name}")
            for m in section.metrics:
                delta_str = ""
                if m.delta_pct is not None:
                    if abs(m.delta_pct) > 999:
                        delta_str = "  --%"
                    else:
                        delta_str = f"{m.delta_pct:+.0f}%"
                    if not m.lower_is_better:
                        # Invert for display: when higher is better,
                        # positive delta means A better
                        pass

                winner_mark = "  "
                if m.winner == "A":
                    winner_mark = "A ✓"
                    wins_a += 1
                elif m.winner == "B":
                    winner_mark = "B ✓"
                    wins_b += 1

                _lower = ""  # removed extra column for cleanliness in narrow format
                line = f"  {m.name:<26s} {m.unit:>5s} {m.label_a:>8s}  {m.label_b:>8s}  {delta_str:>6s}"
                lines.append(line)
                total_metrics += 1

        # Summary (use A/B when labels are identical to avoid confusion)
        lines.append("─" * 72)
        summary_a = label_a_short if label_a_short != label_b_short else "A"
        summary_b = label_b_short if label_a_short != label_b_short else "B"
        lines.append(f"  {summary_a} wins: {wins_a}/{total_metrics}  |  {summary_b} wins: {wins_b}/{total_metrics}")

        # Key findings
        findings = self._key_findings()
        if findings:
            lines.append("")
            for f in findings:
                lines.append(f"  {f}")

        lines.append("═" * 72)
        return "\n".join(lines)

    @property
    def _display_a(self) -> str:
        """Label for display: use short label, A if identical to B."""
        return self.label_a[:12] if self.label_a != self.label_b else "A"

    @property
    def _display_b(self) -> str:
        """Label for display: use short label, B if identical to A."""
        return self.label_b[:12] if self.label_a != self.label_b else "B"

    def _key_findings(self) -> List[str]:
        """Generate bullet-point key findings."""
        findings = []

        # Overall score difference
        rating_a = self.results_a.get('performance_rating', {})
        rating_b = self.results_b.get('performance_rating', {})
        score_a = rating_a.get('score', 0)
        score_b = rating_b.get('score', 0)
        if score_a and score_b:
            diff = score_a - score_b
            if abs(diff) >= 5:
                direction = "higher" if diff > 0 else "lower"
                findings.append(f"* Overall: {self._display_a} score {abs(diff):.0f} pts {direction} ({score_a:.0f} vs {score_b:.0f})")

        # Find largest deltas
        all_metrics = []
        for sec in self.sections:
            for m in sec.metrics:
                if m.delta_pct is not None and m.lower_is_better:
                    all_metrics.append(m)

        # Top 3 improvements (A wins)
        a_better = sorted(
            [m for m in all_metrics if m.winner == "A"],
            key=lambda m: abs(m.delta_pct or 0), reverse=True
        )
        for m in a_better[:3]:
            dp = abs(m.delta_pct or 0)
            dp_str = f"{dp:.0f}%" if dp < 999 else "--"
            findings.append(f"* {self._display_a} better: {m.name.strip()} ({dp_str})")

        # Top 3 regressions (B wins)
        b_better = sorted(
            [m for m in all_metrics if m.winner == "B"],
            key=lambda m: abs(m.delta_pct or 0), reverse=True
        )
        for m in b_better[:3]:
            dp = abs(m.delta_pct or 0)
            dp_str = f"{dp:.0f}%" if dp < 999 else "--"
            findings.append(f"* {self._display_b} better: {m.name.strip()} ({dp_str})")

        return findings[:7]  # cap at 7 findings

    def to_dict(self) -> dict:
        """Export full comparison as structured dict for programmatic use."""
        sections_data = []
        total_metrics = 0
        wins_a = 0
        wins_b = 0

        for sec in self.sections:
            metrics_data = []
            for m in sec.metrics:
                metrics_data.append({
                    'name': m.name.strip(),
                    'unit': m.unit,
                    'value_a': m.value_a,
                    'value_b': m.value_b,
                    'label_a': m.label_a,
                    'label_b': m.label_b,
                    'winner': m.winner,
                    'delta_pct': round(m.delta_pct, 1) if m.delta_pct is not None else None,
                })
                if m.winner == "A":
                    wins_a += 1
                elif m.winner == "B":
                    wins_b += 1
                total_metrics += 1

            sections_data.append({
                'name': sec.name,
                'metrics': metrics_data,
            })

        return {
            'file_a': str(self.file_a),
            'file_b': str(self.file_b),
            'label_a': self.label_a,
            'label_b': self.label_b,
            'duration_a_s': self.results_a.get('flight_info', {}).get('duration_s'),
            'duration_b_s': self.results_b.get('flight_info', {}).get('duration_s'),
            'wins_a': wins_a,
            'wins_b': wins_b,
            'total_metrics': total_metrics,
            'sections': sections_data,
            'findings': self._key_findings(),
            'results_a': self.results_a,
            'results_b': self.results_b,
        }

    def to_json(self) -> str:
        """JSON string output."""
        import json
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False)


# ============================================================
# CLI
# ============================================================

def main():
    p = argparse.ArgumentParser(
        description="Compare two PX4 ULog flight logs side-by-side",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    p.add_argument('file_a', help='First .ulg file')
    p.add_argument('file_b', help='Second .ulg file')
    p.add_argument('--labels', '-l', nargs=2, metavar=('LABEL_A', 'LABEL_B'),
                   help='Custom labels for the two files')
    p.add_argument('--json', action='store_true', help='Output as JSON')
    p.add_argument('--output', '-o', help='Write output to file')
    p.add_argument('-q', '--quiet', action='store_true',
                   help='Suppress progress messages to stderr')

    args = p.parse_args()

    for fp in [args.file_a, args.file_b]:
        if not Path(fp).exists():
            print(f"Error: File not found: {fp}", file=sys.stderr)
            sys.exit(1)

    labels = args.labels or [None, None]

    if not args.quiet:
        size_a = Path(args.file_a).stat().st_size / 1e6
        size_b = Path(args.file_b).stat().st_size / 1e6
        print(f"Loading {Path(args.file_a).name} ({size_a:.0f}MB) ...", file=sys.stderr)
        print(f"Loading {Path(args.file_b).name} ({size_b:.0f}MB) ...", file=sys.stderr)

    t0 = time.time()
    comp = ULogComparer(args.file_a, args.file_b, labels[0] or "", labels[1] or "")
    comp.run()

    if not args.quiet:
        print(f"Done in {time.time() - t0:.1f}s", file=sys.stderr)

    output = comp.to_json() if args.json else comp.to_text()

    if args.output:
        Path(args.output).write_text(output, encoding='utf-8')
        if not args.quiet:
            print(f"Wrote: {args.output}", file=sys.stderr)
    else:
        print(output)


if __name__ == '__main__':
    main()
