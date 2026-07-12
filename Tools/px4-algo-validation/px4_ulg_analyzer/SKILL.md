---
name: px4-ulg-analyzer
description: >
  This skill analyzes PX4 ULog (.ulg) flight log files to evaluate flight
  performance. It parses the ULog binary format directly in pure Python
  (zero dependencies), computes comprehensive performance metrics including
  rate/attitude/position tracking accuracy, vibration levels, EKF health,
  battery consumption, actuator saturation, and flight mode transitions,
  then generates structured reports with an overall performance rating.
  This skill should be used when analyzing PX4 flight logs, evaluating
  flight controller performance, comparing INDI vs PID controllers,
  diagnosing flight issues from ULog data, or generating flight performance
  reports. Triggers include: .ulg file analysis, PX4 log analysis, flight
  performance evaluation, controller comparison, vibration diagnosis.
agent_created: true
---

# PX4 ULog Flight Performance Analyzer

## Overview

This skill provides a complete pipeline for analyzing and **comparing** PX4 autopilot ULog (.ulg)
flight log files. The primary workflow is side-by-side comparison of two logs — e.g., INDI vs
PID controller, different tuning parameters, or hardware variants. It includes a pure-Python ULog
binary parser, a flight performance analyzer, and a comparison tool with both CLI and Python API.

## Architecture

```
scripts/
  ulog_parser.py         ULog binary format parser (FORMAT/ADD_LOGGED_MSG/DATA/INFO/PARAMETER)
  flight_analyzer.py     Performance metrics computation (12 analysis modules)
  compare.py             ★ Side-by-side comparison tool (CLI + Python API)
  report_generator.py    Output formatting (Markdown / JSON / text / AI summary)
  analyze.py             Single-file CLI analysis
  extract_timeseries.py  Extract time-series data as CSV/JSON for plotting
references/
  ulog_topics.md         PX4 ULog topic reference (fields, rates, usage)
  performance_metrics.md Metric definitions, formulas, quality thresholds
assets/
  report_template.md     Report template structure
```

## Pipeline Integration

In the full PX4 algo-validation pipeline, `sitl_sim` collects `.ulg` files to a convention
directory. This skill auto-discovers them:

| Source | Path | Usage |
|--------|------|-------|
| sitl_sim latest | `D:/sitl_logs/latest/` | Most recent simulation |
| sitl_sim archive | `D:/sitl_logs/sitl_<TS>/` | Historical runs |
| Any path | CLI argument | Manual analysis |

```bash
# Directly analyze sitl_sim output
python scripts/analyze.py D:/sitl_logs/latest/*.ulg

# Compare two sitl_sim runs (e.g., INDI vs PID)
python scripts/compare.py \
    D:/sitl_logs/sitl_20260710_170000/*.ulg \
    D:/sitl_logs/sitl_20260710_173000/*.ulg \
    --labels "INDI" "PID"
```

The full pipeline:
```
px4-algo-integration          sitl_sim                  px4_ulg_analyzer
  搜论文 → 改代码 → 编译  →  启仿真 → fly → 收日志  →  解析 → 对比 → 报告
                               └─ D:/sitl_logs/latest/ ──┘
```

## Primary Workflow: Compare Two Logs

### CLI — Side-by-Side Comparison

```bash
# Auto-detect controller type from file names / parameters
python scripts/compare.py indi_flight.ulg pid_flight.ulg

# Custom labels
python scripts/compare.py flight_a.ulg flight_b.ulg --labels "INDI" "PID"

# JSON output for CI/scripting
python scripts/compare.py a.ulg b.ulg --json -o comparison.json
```

The output shows a compact table with every metric compared side-by-side,
including winner per metric (A ✓ / B ✓), delta percentage, and a summary of
key findings. Both controllers are auto-detected from the log parameters.

### Python API — Programmatic Comparison

```python
from compare import ULogComparer

# Quick comparison
comp = ULogComparer("indi.ulg", "pid.ulg")
comp.run()

# Console output
print(comp.to_text())

# Structured data for custom analysis
data = comp.to_dict()
for section in data['sections']:
    for m in section['metrics']:
        if m['winner'] == 'A':
            print(f"  {m['name']}: A={m['label_a']} vs B={m['label_b']}  Δ={m['delta_pct']}%")

# Full results available
results_a = data['results_a']  # dict with all 12 analysis sections
results_b = data['results_b']
```

### Common Comparison Scenarios

**INDI vs PID controller:**
```bash
python scripts/compare.py indi_test.ulg pid_test.ulg --labels "INDI" "PID"
```
Compare rate tracking accuracy, actuator saturation, and vibration robustness.

**Parameter tuning before/after:**
```bash
python scripts/compare.py before.ulg after.ulg --labels "Default" "Tuned"
```
See exactly which metrics improved and by how much.

**Hardware variants:**
```bash
python scripts/compare.py drone_a.ulg drone_b.ulg --labels "Frame A" "Frame B"
```

## Supporting Tools

### Single-File Analysis (diagnostics / quick check)

```bash
# Quick text summary
python scripts/analyze.py <flight.ulg>

# JSON output
python scripts/analyze.py <flight.ulg> -f json -o results.json

# List topics
python scripts/analyze.py <flight.ulg> --topics
```

### Python API (single file)

```python
import sys; sys.path.insert(0, 'scripts')
from ulog_parser import ULogParser
from flight_analyzer import FlightPerformanceAnalyzer

ulog = ULogParser("flight.ulg").parse()
results = FlightPerformanceAnalyzer(ulog).analyze_with_rating()
print(results['performance_rating'])  # {'score': 78, 'grade': 'C', ...}
```

### Time-Series Extraction

```bash
# Extract as CSV for external plotting (matplotlib, PlotJuggler, etc.)
python scripts/extract_timeseries.py <flight.ulg> -t vehicle_attitude -o attitude.csv
python scripts/extract_timeseries.py <flight.ulg> -t vehicle_local_position -s 10 -e 60 --downsample 5 -o pos.csv
```

## Performance Metrics Computed

The analyzer and comparer produce these sections in the results dict:

| Section | Source Topic(s) | Key Metrics |
|---------|----------------|-------------|
| flight_info | all topics | Duration, armed time, topic count |
| attitude | vehicle_attitude | Euler angle stats (roll/pitch/yaw), angular rate stats |
| rate_tracking | vehicle_rates_setpoint vs vehicle_attitude | RMS/mean/max error per axis (deg/s) |
| attitude_tracking | vehicle_attitude_setpoint vs vehicle_attitude | RMS/mean/max error per axis (deg) |
| position_tracking | vehicle_local_position_setpoint vs vehicle_local_position | 3D RMS/mean/max error (m), altitude range |
| vibration | sensor_gyro, sensor_accel | Peak-to-peak, RMS, std per axis; level rating |
| estimator | estimator_status, estimator_innovations | Test ratios, innovations, EKF health |
| battery | battery_status | Voltage profile, current, consumed energy, power |
| flight_modes | vehicle_status | Time per mode, transitions, mode list |
| actuators | actuator_motors or actuator_outputs | Per-motor stats, saturation % |
| controller_config | parameters, info | Controller type (INDI/PID), key gains, aircraft model |
| performance_rating | all above | Overall score (0-100), grade (A-F), issues list |

## Performance Rating System

The analyzer computes an overall performance score (0-100) with grade A-F:

| Grade | Score | Meaning |
|-------|-------|---------|
| A | 90-100 | Excellent, all metrics within range |
| B | 80-89 | Good, minor issues |
| C | 70-79 | Acceptable, some concerns |
| D | 60-69 | Poor, multiple issues need attention |
| F | 0-59 | Critical, flight safety concern |

Score is computed as 100 minus weighted penalties from each metric category.
See `references/performance_metrics.md` for full threshold definitions.

## Key Design Decisions

1. **Zero dependencies**: The parser and analyzer use only Python stdlib. No pyulog, no numpy, no pandas required. This ensures the skill works in any Python environment.

2. **Large file support**: Files >100MB use mmap to avoid loading entire file into memory. The parser scans the binary format in two passes: first for definitions (FORMAT/ADD_LOGGED_MSG), then for data (DATA messages).

3. **Graceful degradation**: Each analysis module checks for topic availability and returns `{'available': False}` if the required topic is missing. The report skips unavailable sections.

4. **Timestamp alignment**: Rate/attitude/position tracking uses nearest-timestamp matching between setpoint and actual data streams, since they may have different logging rates.

## Important Notes

- ULog timestamps are in microseconds. Duration is computed as (last_ts - first_ts) / 1e6.
- Position data uses NED frame: x=North, y=East, z=Down. Altitude = -z.
- Quaternion order is [w, x, y, z] (PX4 convention).
- Flight mode IDs are in `vehicle_status.nav_state`. See `references/ulog_topics.md` section 7.
- **PX4 v1.14+ topic changes**: Angular rates moved from `vehicle_attitude` to `vehicle_angular_velocity` (fields: `xyz[0..2]`). Attitude setpoint uses quaternion `q_d[0..3]` instead of euler angles. The analyzer handles both old and new formats automatically.
- **Parameter format**: PARAMETER messages store the key as `"type name"` (e.g., `"float MC_INDI_ENABLE"`). The first byte is the key length, not a type enum. The parser extracts type and name from the key string.
- **NaN handling**: SITL logs may contain NaN values for fields not applicable in simulation (e.g., `pos_test_ratio`, position setpoints). The analyzer filters NaN/Inf values from all statistics.
- SITL logs may contain additional debug topics (`*_groundtruth`) not present in hardware logs.
- Large files (>100MB) use mmap; parsing a 600MB file takes ~3 minutes.
- Boolean parameters (like MC_INDI_ENABLE) are stored as int32 (0 or 1), not bool.
