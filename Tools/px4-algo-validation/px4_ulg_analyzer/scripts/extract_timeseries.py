#!/usr/bin/env python3
"""
extract_timeseries.py - Extract time-series data from ULog for plotting

Extracts specified topics/fields as CSV or JSON arrays for external
plotting tools (matplotlib, Excel, Grafana, etc.).

Usage:
    python extract_timeseries.py <input.ulg> [options]

Options:
    --topic, -t       Topic name (can repeat for multiple topics)
    --field, -f       Field name within topic (can repeat)
    --output, -o      Output file path
    --format          Output format: csv or json (default: csv)
    --start, -s       Start time offset in seconds (default: 0)
    --end, -e         End time offset in seconds (default: full)
    --downsample      Downsample factor (take every Nth sample, default: 1)
    --list            List all topics with field names and exit

Examples:
    # Extract attitude data as CSV
    python extract_timeseries.py flight.ulg -t vehicle_attitude -o attitude.csv

    # Extract specific fields as JSON
    python extract_timeseries.py flight.ulg -t vehicle_attitude -f rollspeed -f pitchspeed -f yawspeed -f json -o rates.json

    # Extract position data with time window
    python extract_timeseries.py flight.ulg -t vehicle_local_position -s 10 -e 60 -o pos.csv

    # List all available topics
    python extract_timeseries.py flight.ulg --list
"""

import argparse
import sys
import csv
import json
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ulog_parser import ULogParser


def main():
    parser = argparse.ArgumentParser(
        description='Extract time-series data from ULog files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('input', help='Input .ulg file path')
    parser.add_argument('--topic', '-t', action='append', default=[],
                        help='Topic name (repeatable)')
    parser.add_argument('--field', '-f', action='append', default=[],
                        help='Field name within topic (repeatable)')
    parser.add_argument('--output', '-o', help='Output file path')
    parser.add_argument('--format', choices=['csv', 'json'], default='csv',
                        help='Output format (default: csv)')
    parser.add_argument('--start', '-s', type=float, default=0,
                        help='Start time offset in seconds')
    parser.add_argument('--end', '-e', type=float, default=None,
                        help='End time offset in seconds')
    parser.add_argument('--downsample', type=int, default=1,
                        help='Downsample factor (default: 1)')
    parser.add_argument('--list', action='store_true',
                        help='List all topics with field names and exit')

    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: File not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Parsing {input_path.name}...", file=sys.stderr)
    t_start = time.time()
    ulog = ULogParser(str(input_path)).parse()
    print(f"Parsed in {time.time() - t_start:.1f}s", file=sys.stderr)

    # --list: list all topics
    if args.list:
        print("\nAvailable Topics:")
        print("=" * 80)
        for name in sorted(ulog.topics.keys()):
            entries = ulog.topics[name]
            if not entries:
                continue
            count = len(entries)
            duration = (entries[-1].timestamp - entries[0].timestamp) / 1e6
            rate = count / max(duration, 0.001)
            print(f"\n  {name}")
            print(f"    Samples: {count}  Rate: {rate:.1f} Hz  Duration: {duration:.1f}s")
            print(f"    Fields:")
            for key in sorted(entries[0].values.keys()):
                val = entries[0].values[key]
                print(f"      {key}")
        return

    if not args.topic:
        print("Error: No topic specified. Use --topic/-t to specify topics.", file=sys.stderr)
        print("Use --list to see available topics.", file=sys.stderr)
        sys.exit(1)

    # Time window
    t0 = ulog.start_timestamp_us
    start_us = t0 + int(args.start * 1e6)
    end_us = t0 + int(args.end * 1e6) if args.end else ulog.end_timestamp_us

    # Collect data
    all_series = {}

    for topic_name in args.topic:
        if topic_name not in ulog.topics:
            print(f"Warning: Topic not found: {topic_name}", file=sys.stderr)
            continue

        entries = ulog.topics[topic_name]
        series = {}

        for entry in entries:
            if entry.timestamp < start_us or entry.timestamp > end_us:
                continue

            idx = len(series.get('_time_s', []))
            if idx % args.downsample != 0:
                continue

            t_s = (entry.timestamp - t0) / 1e6

            if '_time_s' not in series:
                series['_time_s'] = []
            series['_time_s'].append(round(t_s, 6))

            for key, val in entry.values.items():
                # Filter by --field if specified
                if args.field:
                    field_names = [f.split('.')[-1] for f in args.field]
                    if key not in field_names:
                        continue

                col_name = f"{topic_name}.{key}"
                if col_name not in series:
                    series[col_name] = []
                series[col_name].append(val)

        all_series.update(series)

    if not all_series:
        print("No data found for the specified criteria.", file=sys.stderr)
        sys.exit(1)

    # Remove columns that don't match the time column length
    time_len = len(all_series.get('_time_s', []))
    for key in list(all_series.keys()):
        if len(all_series[key]) != time_len:
            del all_series[key]

    # Output
    output = ''
    if args.format == 'csv':
        lines = []
        headers = list(all_series.keys())
        lines.append(','.join(headers))
        for i in range(time_len):
            row = []
            for h in headers:
                val = all_series[h][i]
                if isinstance(val, float):
                    row.append(f"{val:.6f}")
                else:
                    row.append(str(val))
            lines.append(','.join(row))
        output = '\n'.join(lines)

    elif args.format == 'json':
        records = []
        for i in range(time_len):
            record = {}
            for h in all_series:
                record[h] = all_series[h][i]
            records.append(record)
        output = json.dumps(records, indent=2)

    if args.output:
        Path(args.output).write_text(output, encoding='utf-8')
        print(f"Output written to: {args.output} ({time_len} samples)", file=sys.stderr)
    else:
        print(output)

    print(f"Extracted {time_len} samples, {len(all_series)} columns", file=sys.stderr)


if __name__ == '__main__':
    main()
