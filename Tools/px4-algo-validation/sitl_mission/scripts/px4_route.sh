#!/bin/bash
# =============================================================================
# px4_route.sh - 执行多航点航线任务
# =============================================================================
# 用法：
#   ./px4_route.sh <mission_profile.json>
#   ./px4_route.sh ../missions/scan_area.json
#
# 前提：PX4 SITL 必须已经运行 (tmux session px4_mission)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${PX4_DIR:-/mnt/d/source/PX4-Autopilot-AllInOne}"
VENV="${PX4_DIR}/.venv"

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <profile.json>"
    echo ""
    echo "  Example: $0 ../missions/scan_area.json"
    echo ""
    echo "  Mission profiles are JSON files defining waypoints."
    echo "  See missions/ directory for examples."
    exit 1
fi

PROFILE="$1"
if [ ! -f "$PROFILE" ]; then
    echo "ERROR: Profile not found: $PROFILE"
    exit 1
fi

if [ ! -f "$VENV/bin/python3" ]; then
    echo "ERROR: Python venv not found at $VENV"
    echo "Run: cd $PX4_DIR && python3 -m venv .venv && .venv/bin/pip install pymavlink"
    exit 1
fi

exec "$VENV/bin/python3" "$SCRIPT_DIR/px4_mission_runner.py" route "$PROFILE"
