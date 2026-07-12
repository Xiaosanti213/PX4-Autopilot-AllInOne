#!/bin/bash
# =============================================================================
# px4_goto.sh - 飞往指定 NED 坐标
# =============================================================================
# 用法：
#   ./px4_goto.sh X Y Z [yaw]
#   ./px4_goto.sh 20 0 -10          # 飞往前 20m、下 10m (上升)
#   ./px4_goto.sh 10 5 -15 1.57     # 带偏航角
#
# 前提：PX4 SITL 必须已经运行 (tmux session px4_mission)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${PX4_DIR:-/mnt/d/source/PX4-Autopilot-AllInOne}"
VENV="${PX4_DIR}/.venv"

if [ "$#" -lt 3 ]; then
    echo "Usage: $0 X Y Z [yaw]"
    echo ""
    echo "  X/Y/Z = NED 坐标 (米), Z 为负表示上升"
    echo "  yaw   = 偏航角 (弧度), 可选"
    echo ""
    echo "  Example: $0 20 0 -10  (飞往前 20m，上升 10m)"
    exit 1
fi

if [ ! -f "$VENV/bin/python3" ]; then
    echo "ERROR: Python venv not found at $VENV"
    echo "Run: cd $PX4_DIR && python3 -m venv .venv && .venv/bin/pip install pymavlink"
    exit 1
fi

exec "$VENV/bin/python3" "$SCRIPT_DIR/px4_mission_runner.py" goto "$@"
