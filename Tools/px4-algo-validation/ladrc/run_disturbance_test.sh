#!/bin/bash
# =============================================================================
# run_disturbance_test.sh - LADRC vs PID 多阶段扰动对比测试
# =============================================================================
# 在 SIH 仿真中分阶段注入扰动，测试控制器的抗扰能力
#
# 阶段设计:
#   1. 起飞悬停 (0-8s)    — 建立基线
#   2. 恒定侧风 (8-18s)    — SIH_WIND_N=8 m/s
#   3. 叠加质量翻倍 (18-28s) — SIH_MASS=2.0 kg
#   4. 叠加推力衰减 (28-38s) — SIH_T_MAX=3.0 N (降低40%)
#   5. 移除所有扰动 (38-48s) — 观察恢复能力
#   6. 降落
#
# Usage: ./run_disturbance_test.sh [pid|ladrc]
# =============================================================================
set -e

# Auto-detect correct path (WSL /mnt/d/ vs MinGW /d/)
if [ -d "/mnt/d/source/PX4-Autopilot-AllInOne" ]; then
    PX4_DIR="/mnt/d/source/PX4-Autopilot-AllInOne"
elif [ -d "/d/source/PX4-Autopilot-AllInOne" ]; then
    PX4_DIR="/d/source/PX4-Autopilot-AllInOne"
else
    echo "ERROR: Cannot find PX4 directory!"
    exit 1
fi
MODE="${1:-pid}"
SESSION="px4_dist_${MODE}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"

log()   { echo "[$(date +%H:%M:%S)] [${MODE^^}] $1"; }
stage() { echo ""; echo "############################################"; echo "# $1"; echo "############################################"; echo ""; }

# =============================================================================
cleanup() {
    echo ""
    log "=== Cleanup ==="
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux send-keys -t "$SESSION" "commander disarm" Enter 2>/dev/null || true
        sleep 1
        tmux send-keys -t "$SESSION" C-c 2>/dev/null || true
        sleep 2
        tmux kill-session -t "$SESSION" 2>/dev/null || true
        log "Cleaned up tmux session"
    fi
    pkill -f "bin/px4" 2>/dev/null || true
}
trap cleanup EXIT

# =============================================================================
# Pre-flight cleanup
# =============================================================================
tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -f "bin/px4" 2>/dev/null || true
sleep 2

# =============================================================================
# Stage 0: Start PX4 SITL
# =============================================================================
stage "STAGE 0: Starting PX4 SITL (SIH quadx)"

# Remove old raw ULG session directories (timestamp-pattern) to keep log dir clean
rm -rf "$PX4_DIR/build/px4_sitl_default/rootfs/log/20"* 2>/dev/null || true
# Record timestamp BEFORE flight to correctly identify new ULGs
TIMESTAMP_BEFORE=$(date +%s)

tmux new-session -d -s "$SESSION" \
    "cd $PX4_DIR && PX4_SIM_MODEL=sihsim_quadx $PX4_BIN 2>&1"

log "Waiting for PX4 to initialize (20s)..."
sleep 20

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    log "ERROR: PX4 failed to start!"
    exit 1
fi

# Test responsiveness
tmux send-keys -t "$SESSION" "param show SIH_MASS" Enter
sleep 2

# =============================================================================
# Stage 0.5: Set controller parameters
# =============================================================================
log "=== Setting parameters ==="

# Common params
tmux send-keys -t "$SESSION" "param set NAV_DLL_ACT 0" Enter; sleep 1
tmux send-keys -t "$SESSION" "param set MIS_TAKEOFF_ALT 5" Enter; sleep 1

if [ "$MODE" = "ladrc" ]; then
    log "Configuring LADRC..."
    tmux send-keys -t "$SESSION" "param set MC_LADRC_ENABLE 1" Enter; sleep 0.5
    # Conservative parameters - prioritize stability
    tmux send-keys -t "$SESSION" "param set MC_LADRC_B0 60.0" Enter; sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_B0_Y 25.0" Enter; sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_WO 30.0" Enter; sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_WO_Y 20.0" Enter; sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_WC 8.0" Enter; sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_WC_Y 5.0" Enter; sleep 0.5
    log "LADRC params: b0=60/25, wo=30/20, wc=8/5"
else
    log "Using default PID controller"
    tmux send-keys -t "$SESSION" "param set MC_LADRC_ENABLE 0" Enter; sleep 1
fi

# =============================================================================
# Stage 1: Takeoff + baseline hover
# =============================================================================
stage "STAGE 1: Takeoff & Baseline Hover (0-8s)"
T_STAGE1=0

log "Arming..."
tmux send-keys -t "$SESSION" "commander arm" Enter
sleep 3

log "Takeoff to 5m..."
tmux send-keys -t "$SESSION" "commander takeoff" Enter
sleep 8

log "Stage 1 complete - stable hover established"

# =============================================================================
# Stage 2: Side wind 8 m/s
# =============================================================================
stage "STAGE 2: Side Wind 8 m/s (8-18s)"
T_STAGE2=$(date +%s)

log "Injecting wind: SIH_WIND_N = 8.0 m/s"
tmux send-keys -t "$SESSION" "param set SIH_WIND_N 8.0" Enter

for i in $(seq 1 5); do
    sleep 2
    tmux send-keys -t "$SESSION" "listener vehicle_local_position 1" Enter
    sleep 0.5
    output=$(tmux capture-pane -t "$SESSION" -p -S -5 2>/dev/null)
    z=$(echo "$output" | grep "^\s*z:" | tail -1 | awk '{print $2}')
    log "  [wind t=${i}x2s] z=${z}"
done

log "Stage 2 complete - wind active"

# =============================================================================
# Stage 3: Mass increase (on top of wind)
# =============================================================================
stage "STAGE 3: Mass x2 (2.0 kg) + Wind (18-28s)"

log "Setting SIH_MASS = 2.0 kg (was 1.0)"
tmux send-keys -t "$SESSION" "param set SIH_MASS 2.0" Enter

for i in $(seq 1 5); do
    sleep 2
    tmux send-keys -t "$SESSION" "listener vehicle_local_position 1" Enter
    sleep 0.5
    output=$(tmux capture-pane -t "$SESSION" -p -S -5 2>/dev/null)
    z=$(echo "$output" | grep "^\s*z:" | tail -1 | awk '{print $2}')
    log "  [mass+wind t=${i}x2s] z=${z}"
done

log "Stage 3 complete - doubled mass"

# =============================================================================
# Stage 4: Thrust degradation (on top of wind + mass)
# =============================================================================
stage "STAGE 4: Thrust -40% (T_MAX=3.0) + Mass + Wind (28-38s)"

log "Setting SIH_T_MAX = 3.0 N (was 5.0, -40%)"
tmux send-keys -t "$SESSION" "param set SIH_T_MAX 3.0" Enter

for i in $(seq 1 5); do
    sleep 2
    tmux send-keys -t "$SESSION" "listener vehicle_local_position 1" Enter
    sleep 0.5
    output=$(tmux capture-pane -t "$SESSION" -p -S -5 2>/dev/null)
    z=$(echo "$output" | grep "^\s*z:" | tail -1 | awk '{print $2}')
    log "  [thrust-40% t=${i}x2s] z=${z}"
done

log "Stage 4 complete - worst case scenario"

# =============================================================================
# Stage 5: Remove ALL disturbances - recovery
# =============================================================================
stage "STAGE 5: Recovery - Remove All Disturbances (38-48s)"

log "Restoring nominal conditions..."
tmux send-keys -t "$SESSION" "param set SIH_WIND_N 0.0" Enter; sleep 1
tmux send-keys -t "$SESSION" "param set SIH_MASS 1.0" Enter; sleep 1
tmux send-keys -t "$SESSION" "param set SIH_T_MAX 5.0" Enter; sleep 1

for i in $(seq 1 5); do
    sleep 2
    tmux send-keys -t "$SESSION" "listener vehicle_local_position 1" Enter
    sleep 0.5
    output=$(tmux capture-pane -t "$SESSION" -p -S -5 2>/dev/null)
    z=$(echo "$output" | grep "^\s*z:" | tail -1 | awk '{print $2}')
    log "  [recovery t=${i}x2s] z=${z}"
done

log "Stage 5 complete - disturbances removed"

# =============================================================================
# Stage 6: Land
# =============================================================================
stage "STAGE 6: Landing"

log "Initiating landing..."
tmux send-keys -t "$SESSION" "commander land" Enter
sleep 15

log "Disarming..."
tmux send-keys -t "$SESSION" "commander disarm" Enter
sleep 3

# =============================================================================
# Extract ULG log
# =============================================================================
log "=== Extracting ULG log ==="
sleep 2

# Find ULG files created after flight start
LOG_DIR="$PX4_DIR/build/px4_sitl_default/rootfs/log"
ulg_file=$(find "$LOG_DIR" -name "*.ulg" -newermt "@${TIMESTAMP_BEFORE}" 2>/dev/null | head -1)

if [ -z "$ulg_file" ]; then
    # Fallback: find any ULG in session directories created after flight start
    ulg_file=$(find "$LOG_DIR" -path "*/20*/*.ulg" -newermt "@${TIMESTAMP_BEFORE}" 2>/dev/null | head -1)
fi

if [ -z "$ulg_file" ]; then
    # Last resort: find the most recently modified ULG
    ulg_file=$(find "$LOG_DIR" -name "*.ulg" -printf "%T@ %p\n" 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)
fi

if [ -n "$ulg_file" ]; then
    file_size=$(du -h "$ulg_file" | awk '{print $1}')
    log "ULG log: $ulg_file ($file_size)"
    cp "$ulg_file" "$PX4_DIR/build/px4_sitl_default/rootfs/log/${MODE}_disturbance.ulg"
    log "Copied to: ${MODE}_disturbance.ulg"
    echo ""
    echo "============================================"
    echo "  TEST COMPLETE: $MODE"
    echo "  ULG: $(basename "$ulg_file") ($file_size)"
    echo "  Copy: ${MODE}_disturbance.ulg"
    echo "============================================"
else
    log "ERROR: No ULG file found!"
    find "$PX4_DIR/build/px4_sitl_default/rootfs/log" -name "*.ulg" -exec ls -la {} \;
fi
