#!/bin/bash
# =============================================================================
# px4_send_cmd.sh - 向运行中的 PX4 SITL 发送命令
# =============================================================================
# 用法：
#   ./px4_send_cmd.sh "commander check"
#   ./px4_send_cmd.sh "listener vehicle_local_position" --output
#
# 环境变量：
#   SESSION  tmux session 名称（默认 px4_mission）
# =============================================================================

SESSION="${SESSION:-px4_mission}"
OUTPUT_MODE=0

if [ "$1" = "--output" ] || [ "$2" = "--output" ]; then
    OUTPUT_MODE=1
    if [ "$1" = "--output" ]; then
        shift
    fi
fi

CMD="${1:-commander check}"

# 检查 session 是否存在
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "ERROR: tmux session '$SESSION' 不存在"
    echo "请先启动 PX4 SITL: bash px4_auto_mission.sh"
    exit 1
fi

# 发送命令
tmux send-keys -t "$SESSION" "$CMD" Enter

if [ "$OUTPUT_MODE" = "1" ]; then
    sleep 2
    echo "=== Output ==="
    tmux capture-pane -t "$SESSION" -p -S -30
fi

echo "Command sent: $CMD"
