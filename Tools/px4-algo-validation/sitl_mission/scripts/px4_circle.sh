#!/bin/bash
# =============================================================================
# px4_circle.sh - 飞圆圈航线
# =============================================================================
# 用法：
#   ./px4_circle.sh [radius] [alt] [turns]
#   ./px4_circle.sh 15 10 3          # 半径 15m, 高度 10m, 转 3 圈
#   ./px4_circle.sh 20               # 半径 20m, 默认高度 10m, 3 圈
#
# 前提：PX4 SITL 必须已经运行 (tmux session px4_mission)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${PX4_DIR:-/mnt/d/source/PX4-Autopilot-AllInOne}"
VENV="${PX4_DIR}/.venv"

if [ ! -f "$VENV/bin/python3" ]; then
    echo "ERROR: Python venv not found at $VENV"
    echo "Run: cd $PX4_DIR && python3 -m venv .venv && .venv/bin/pip install pymavlink"
    exit 1
fi

exec "$VENV/bin/python3" "$SCRIPT_DIR/px4_mission_runner.py" circle "$@"
