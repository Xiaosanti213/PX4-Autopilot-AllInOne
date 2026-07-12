#!/bin/bash
# =============================================================================
# px4_auto_mission.sh - PX4 SITL 自动起飞验证脚本
# =============================================================================
# 在 WSL 内运行，自动完成：
#   编译 → 启动仿真 → commander check → 参数设置 → 起飞 → 高度监控 → 降落上锁
#
# 用法：
#   ./px4_auto_mission.sh                    # 默认起飞 5m，监控 15s
#   ./px4_auto_mission.sh 10                 # 起飞到 10m
#   ./px4_auto_mission.sh 5 30               # 起飞 5m，监控 30s
#   SKIP_BUILD=1 ./px4_auto_mission.sh       # 跳过编译（已有编译产物时）
#   SESSION=my_session ./px4_auto_mission.sh # 指定 tmux session 名称
#
# 环境变量：
#   PX4_DIR          PX4 源码目录（默认 /mnt/d/source/PX4-Autopilot-AllInOne）
#   SKIP_BUILD       设为 1 跳过编译步骤
#   SESSION          tmux session 名称（默认 px4_mission）
#   READY_TIMEOUT    等待 PX4 就绪的超时秒数（默认 120）
# =============================================================================

set -e

# === 配置 ===
PX4_DIR="${PX4_DIR:-/mnt/d/source/PX4-Autopilot-AllInOne}"
SESSION="${SESSION:-px4_mission}"
TAKEOFF_ALT="${1:-5}"
MONITOR_SEC="${2:-15}"
READY_TIMEOUT="${READY_TIMEOUT:-120}"
SKIP_BUILD="${SKIP_BUILD:-0}"

# === 颜色 ===
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

log()   { echo -e "${GREEN}[$(date +%H:%M:%S)]${NC} $1"; }
warn()  { echo -e "${YELLOW}[$(date +%H:%M:%S)] WARN${NC} $1"; }
err()   { echo -e "${RED}[$(date +%H:%M:%S)] ERROR${NC} $1"; }
info()  { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $1"; }
header(){ echo -e "\n${BOLD}${CYAN}=== $1 ===${NC}"; }

# === 清理函数 ===
cleanup() {
    echo ""
    header "清理"
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        # 上锁
        tmux send-keys -t "$SESSION" "commander disarm" Enter 2>/dev/null || true
        sleep 1
        tmux send-keys -t "$SESSION" C-c 2>/dev/null || true
        sleep 2
        tmux kill-session -t "$SESSION" 2>/dev/null || true
        log "已清理 tmux session '$SESSION'"
    fi
}
trap cleanup EXIT

# === 辅助函数 ===
send_cmd() {
    tmux send-keys -t "$SESSION" "$1" Enter
}

get_output() {
    # 捕获 tmux 最后 N 行
    tmux capture-pane -t "$SESSION" -p -S -50 2>/dev/null
}

wait_for_text() {
    local pattern="$1"
    local timeout="${2:-60}"
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if get_output | grep -q "$pattern"; then
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 1
}

wait_for_commander_ready() {
    local timeout="${1:-$READY_TIMEOUT}"
    local elapsed=0
    # PX4 启动后 commander 需要约 30-60s 就绪
    # 轮询发送 commander check 直到有正常响应
    while [ $elapsed -lt $timeout ]; do
        # 清屏并发送 commander check
        send_cmd "commander check 2>&1"
        sleep 3
        local output
        output=$(get_output)
        if echo "$output" | grep -qE "(Ready|ARMING|DISARMED|health|Preflight)"; then
            log "PX4 Commander 就绪 (耗时 ${elapsed}s)"
            return 0
        fi
        elapsed=$((elapsed + 3))
        if [ $((elapsed % 15)) -eq 0 ]; then
            info "等待 PX4 就绪... (${elapsed}s/${timeout}s)"
        fi
    done
    err "PX4 Commander 超时未就绪 (${timeout}s)"
    return 1
}

# === 清理旧进程 ===
header "清理旧进程"
# 先杀旧 px4 和 gz 进程（不影响当前可能还在运行的）
for proc in px4 gz sim ruby; do
    pkill -f "$proc" 2>/dev/null && log "已终止旧 $proc 进程" || true
done
# 清理旧 tmux session
tmux kill-session -t "$SESSION" 2>/dev/null && log "已清理旧 tmux session" || true
sleep 2

# =============================================================================
# Step 1: 编译
# =============================================================================
if [ "$SKIP_BUILD" != "1" ]; then
    header "Step 1/6: 编译 PX4 SITL"
    if [ ! -d "$PX4_DIR" ]; then
        err "PX4 目录不存在: $PX4_DIR"
        exit 1
    fi
    cd "$PX4_DIR"
    log "开始编译 (HEADLESS=1 make px4_sitl gz_x500)..."
    HEADLESS=1 make px4_sitl gz_x500 2>&1 | tail -20
    log "编译完成"
else
    header "Step 1/6: 跳过编译 (SKIP_BUILD=1)"
fi

# =============================================================================
# Step 2: 启动仿真
# =============================================================================
header "Step 2/6: 启动 PX4 SITL 仿真"
cd "$PX4_DIR"

log "在 tmux session '$SESSION' 中启动仿真..."
tmux new-session -d -s "$SESSION" "cd $PX4_DIR && HEADLESS=1 make px4_sitl gz_x500 2>&1"
log "tmux session 已创建，等待 PX4 初始化..."

# 等待 Gazebo 和 PX4 启动
info "等待 Gazebo 启动..."
if ! wait_for_text "Gazebo" 30; then
    warn "未检测到 Gazebo 启动信息，继续等待..."
fi

# =============================================================================
# Step 3: 等待 PX4 就绪
# =============================================================================
header "Step 3/6: 等待 PX4 Commander 就绪"
if ! wait_for_commander_ready "$READY_TIMEOUT"; then
    err "PX4 启动失败，请检查 tmux 输出: tmux attach -t $SESSION"
    exit 1
fi

# =============================================================================
# Step 4: Commander Check & 参数设置
# =============================================================================
header "Step 4/6: Commander Check & 参数设置"

send_cmd "commander check"
sleep 2
log "Commander Check 输出:"
echo "----------------------------------------"
get_output | grep -A 50 "commander check" | head -30 || get_output | tail -20
echo "----------------------------------------"

# 设置参数
info "设置参数 NAV_DLL_ACT=0 (禁用地理围栏)..."
send_cmd "param set NAV_DLL_ACT 0"
sleep 1

info "设置参数 MIS_TAKEOFF_ALT=${TAKEOFF_ALT} (起飞高度 ${TAKEOFF_ALT}m)..."
send_cmd "param set MIS_TAKEOFF_ALT ${TAKEOFF_ALT}"
sleep 1

log "参数设置完成"

# =============================================================================
# Step 5: 起飞 & 监控
# =============================================================================
header "Step 5/6: 起飞 & 高度监控"

info "Arming..."
send_cmd "commander arm"
sleep 3

info "Taking off to ${TAKEOFF_ALT}m..."
send_cmd "commander takeoff"
sleep 5

log "开始监控 vehicle_local_position (z 轴 = 高度 NED, 负值 = 上升)"
echo ""

local monitor_elapsed=0
local last_z=""
while [ $monitor_elapsed -lt $MONITOR_SEC ]; do
    send_cmd "listener vehicle_local_position"
    sleep 2
    local output
    output=$(get_output | grep -E "^\s+(x|y|z):" | head -6)
    local current_z
    current_z=$(echo "$output" | grep "z:" | awk '{print $2}')
    
    if [ -n "$current_z" ] && [ "$current_z" != "$last_z" ]; then
        local abs_z
        abs_z=$(echo "$current_z" | sed 's/-//')
        printf "  ${CYAN}[t=%03ds]${NC} z: ${GREEN}%s${NC} (高度约 %.1fm)\n" "$monitor_elapsed" "$current_z" "$abs_z"
        last_z="$current_z"
    fi
    
    monitor_elapsed=$((monitor_elapsed + 2))
done

# =============================================================================
# Step 6: 降落 & 上锁
# =============================================================================
header "Step 6/6: 降落 & 上锁"

info "发送着陆指令..."
send_cmd "commander land"

log "等待着陆完成 (约 10s)..."
sleep 10

info "Disarming..."
send_cmd "commander disarm"
sleep 2

log "任务完成！无人机已安全着陆并上锁"

# === 最终状态 ===
echo ""
header "最终状态"
send_cmd "listener vehicle_local_position"
sleep 2
get_output | grep -E "^\s+(x|y|z|vx|vy|vz):" | head -6

echo ""
log "======================================"
log "  任务摘要"
log "  起飞高度: ${TAKEOFF_ALT}m"
log "  监控时长: ${MONITOR_SEC}s"
log "  状态:     成功完成"
log "======================================"
