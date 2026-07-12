#!/usr/bin/env python3
"""
analyze.py - CLI entry point for PX4 ULog flight performance analysis

Usage:
    python analyze.py <input.ulg> [options]

Options:
    --format, -f    Output format: text, markdown, json, ai (default: text)
    --output, -o    Write to file instead of stdout
    --topics        List available topics and exit
    --topic         Show details for a specific topic
    --no-rating     Skip performance rating computation

Examples:
    # Quick text summary
    python analyze.py flight.ulg

    # Full markdown report
    python analyze.py flight.ulg -f markdown -o report.md

    # JSON output
    python analyze.py flight.ulg -f json -o results.json

    # List all topics
    python analyze.py flight.ulg --topics

    # AI summary
    python analyze.py flight.ulg -f ai
"""

import argparse
import sys
import os
import time
from pathlib import Path

# Add parent dir to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from ulog_parser import ULogParser
from flight_analyzer import FlightPerformanceAnalyzer
from report_generator import ReportGenerator


def main():
    parser = argparse.ArgumentParser(
        description='PX4 ULog Flight Performance Analyzer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('input', help='Input .ulg file path')
    parser.add_argument(
        '--format', '-f',
        choices=['text', 'markdown', 'json', 'ai'],
        default='text',
        help='Output format (default: text)'
    )
    parser.add_argument(
        '--output', '-o',
        help='Output file path (default: stdout)'
    )
    parser.add_argument(
        '--topics',
        action='store_true',
        help='List all available topics and exit'
    )
    parser.add_argument(
        '--topic',
        help='Show field details for a specific topic'
    )
    parser.add_argument(
        '--no-rating',
        action='store_true',
        help='Skip performance rating computation (faster)'
    )

    args = parser.parse_args()

    # Validate input
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: File not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    if not input_path.suffix == '.ulg':
        print(f"Warning: File does not have .ulg extension: {input_path}", file=sys.stderr)

    # Parse ULog
    file_size_mb = input_path.stat().st_size / (1024 * 1024)
    print(f"Parsing {input_path.name} ({file_size_mb:.1f} MB)...", file=sys.stderr)

    t_start = time.time()
    try:
        ulog = ULogParser(str(input_path)).parse()
    except Exception as e:
        print(f"Error: Failed to parse ULog: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)

    t_parse = time.time() - t_start
    print(f"Parsed in {t_parse:.1f}s: {len(ulog.topics)} topics, "
          f"{ulog.duration_s:.1f}s flight duration", file=sys.stderr)

    # --topics: list and exit
    if args.topics:
        print("\nAvailable Topics:")
        print("-" * 60)
        for name in sorted(ulog.topics.keys()):
            count = len(ulog.topics[name])
            print(f"  {name:50s} {count:>8} samples")
        print("-" * 60)
        print(f"Total: {len(ulog.topics)} topics")
        return

    # --topic: show topic details
    if args.topic:
        topic_name = args.topic
        if topic_name not in ulog.topics:
            print(f"Topic not found: {topic_name}", file=sys.stderr)
            print(f"Available: {', '.join(sorted(ulog.topics.keys())[:20])}...", file=sys.stderr)
            sys.exit(1)

        entries = ulog.topics[topic_name]
        print(f"\nTopic: {topic_name}")
        print(f"Samples: {len(entries)}")
        print(f"Duration: {(entries[-1].timestamp - entries[0].timestamp) / 1e6:.1f}s")
        print(f"Rate: {len(entries) / max((entries[-1].timestamp - entries[0].timestamp) / 1e6, 0.001):.1f} Hz")
        print(f"\nFields:")
        if entries:
            for key in sorted(entries[0].values.keys()):
                val = entries[0].values[key]
                val_type = type(val).__name__
                print(f"  {key:40s} ({val_type})")
        return

    # Analyze
    print("Analyzing flight performance...", file=sys.stderr)
    t_start = time.time()

    analyzer = FlightPerformanceAnalyzer(ulog)
    if args.no_rating:
        results = analyzer.analyze()
    else:
        results = analyzer.analyze_with_rating()

    t_analyze = time.time() - t_start
    print(f"Analysis complete in {t_analyze:.1f}s", file=sys.stderr)

    # Generate output
    gen = ReportGenerator(results, ulog)
    output = ''

    if args.format == 'text':
        output = gen.to_text()
    elif args.format == 'markdown':
        output = gen.to_markdown(title=f"ULog Analysis: {input_path.stem}")
    elif args.format == 'json':
        output = gen.to_json()
    elif args.format == 'ai':
        output = gen.to_ai_summary()

    # Write output
    if args.output:
        Path(args.output).write_text(output, encoding='utf-8')
        print(f"Output written to: {args.output}", file=sys.stderr)
    else:
        print(output)


if __name__ == '__main__':
    main()
