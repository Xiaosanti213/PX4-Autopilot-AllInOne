#!/bin/bash
# =============================================================================
# run_ladrc_flight.sh - LADRC vs PID SITL flight comparison
# =============================================================================
# Runs PX4 SITL with SIH backend (no Gazebo needed)
# Usage: ./run_ladrc_flight.sh [pid|ladrc] [monitor_sec]
# =============================================================================
set -e

PX4_DIR="/mnt/d/source/PX4-Autopilot-AllInOne"
MODE="${1:-ladrc}"
MONITOR_SEC="${2:-25}"
SESSION="px4_${MODE}_flight"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"

log()   { echo "[$(date +%H:%M:%S)] $1"; }

cleanup() {
    echo ""
    log "=== Cleanup ==="
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux send-keys -t "$SESSION" "commander disarm" Enter 2>/dev/null || true
        sleep 1
        tmux send-keys -t "$SESSION" C-c 2>/dev/null || true
        sleep 1
        tmux kill-session -t "$SESSION" 2>/dev/null || true
        log "Cleaned up tmux session"
    fi
    # Kill any lingering px4 processes
    pkill -f "bin/px4" 2>/dev/null || true
}
trap cleanup EXIT

# Clean up any old sessions
tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -f "bin/px4" 2>/dev/null || true
sleep 2

# === Start PX4 SITL with SIH ===
log "=== Starting PX4 SITL (SIH quadx, MODE=$MODE) ==="
tmux new-session -d -s "$SESSION" \
    "cd $PX4_DIR && PX4_SIM_MODEL=sihsim_quadx $PX4_BIN 2>&1"

log "Waiting for PX4 to initialize..."
sleep 15

# === Check if PX4 is running ===
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    log "ERROR: PX4 failed to start"
    exit 1
fi

# Send a test command to check responsiveness
tmux send-keys -t "$SESSION" "param show MC_LADRC_ENABLE" Enter
sleep 3

# === Set parameters ===
log "=== Setting parameters ==="

# Disable geofence
tmux send-keys -t "$SESSION" "param set NAV_DLL_ACT 0" Enter
sleep 1

# Set takeoff altitude
tmux send-keys -t "$SESSION" "param set MIS_TAKEOFF_ALT 5" Enter
sleep 1

if [ "$MODE" = "ladrc" ]; then
    log "Enabling LADRC..."
    tmux send-keys -t "$SESSION" "param set MC_LADRC_ENABLE 1" Enter
    sleep 1
    # Fast observer to quickly compensate disturbances (like PID integral)
    # b0=67 → wc/b0=0.15 (matches PID P gain exactly)
    # wo=40 → fast z3 convergence (time const ~0.025s, like PID integral)
    tmux send-keys -t "$SESSION" "param set MC_LADRC_B0 67.0" Enter
    sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_B0_Y 30.0" Enter
    sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_WO 40.0" Enter
    sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_WO_Y 25.0" Enter
    sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_WC 10.0" Enter
    sleep 0.5
    tmux send-keys -t "$SESSION" "param set MC_LADRC_WC_Y 6.0" Enter
    sleep 1
    log "LADRC enabled (b0=67/30, wo=40/25, wc=10/6)"
else
    log "Using default PID controller"
    tmux send-keys -t "$SESSION" "param set MC_LADRC_ENABLE 0" Enter
    sleep 1
fi

# === Arm and takeoff ===
log "=== Arming ==="
tmux send-keys -t "$SESSION" "commander arm" Enter
sleep 3

log "=== Taking off to 5m ==="
tmux send-keys -t "$SESSION" "commander takeoff" Enter
sleep 5

# === Monitor flight ===
log "=== Monitoring flight for ${MONITOR_SEC}s ==="
elapsed=0
while [ $elapsed -lt $MONITOR_SEC ]; do
    tmux send-keys -t "$SESSION" "listener vehicle_local_position 1" Enter
    sleep 2
    # Capture output
    output=$(tmux capture-pane -t "$SESSION" -p -S -30 2>/dev/null)
    z_val=$(echo "$output" | grep "^\s*z:" | tail -1 | awk '{print $2}')
    if [ -n "$z_val" ]; then
        alt=$(echo "$z_val" | sed 's/-//')
        log "  [t=${elapsed}s] altitude: ${alt}m (z=${z_val})"
    fi
    elapsed=$((elapsed + 2))
done

# === Land ===
log "=== Landing ==="
tmux send-keys -t "$SESSION" "commander land" Enter
sleep 15

log "=== Disarming ==="
tmux send-keys -t "$SESSION" "commander disarm" Enter
sleep 2

# === Get log file path ===
log "=== Finding ULG log ==="
sleep 2
# Find the most recent ULG file
ulg_file=$(find "$PX4_DIR/build/px4_sitl_default/rootfs/log" -name "*.ulg" -newer "$PX4_DIR/build/px4_sitl_default/bin/px4" 2>/dev/null | sort | tail -1)
if [ -z "$ulg_file" ]; then
    # Fallback: find most recent ULG
    ulg_file=$(find "$PX4_DIR/build/px4_sitl_default/rootfs/log" -name "*.ulg" 2>/dev/null | sort | tail -1)
fi

if [ -n "$ulg_file" ]; then
    file_size=$(du -h "$ulg_file" | awk '{print $1}')
    log "ULG log found: $ulg_file ($file_size)"
    echo "ULG_FILE=$ulg_file"
    # Copy to a named location
    cp "$ulg_file" "$PX4_DIR/build/px4_sitl_default/rootfs/log/${MODE}_flight.ulg"
    log "Copied to: $PX4_DIR/build/px4_sitl_default/rootfs/log/${MODE}_flight.ulg"
else
    log "WARNING: No ULG file found!"
    # List all logs
    find "$PX4_DIR/build/px4_sitl_default/rootfs/log" -name "*.ulg" -exec ls -la {} \;
fi

log "=== Flight complete (MODE=$MODE) ==="
