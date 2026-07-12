#!/usr/bin/env python3
# =============================================================================
# px4_mission_runner.py - PX4 SITL 通用任务执行器
# =============================================================================
# 通过 MAVLink 连接 PX4 SITL，执行各类飞行任务。
#
# 用法：
#   python3 px4_mission_runner.py hover [alt] [duration]
#   python3 px4_mission_runner.py goto X Y Z [yaw]
#   python3 px4_mission_runner.py circle radius [alt] [turns]
#   python3 px4_mission_runner.py route waypoints.json
#
# 环境变量：
#   MAVLINK_PORT    MAVLink UDP 端口（默认 14540，对应 SITL offboard 端口）
#   PX4_VENV        Python venv 路径（默认 .venv）
# =============================================================================

import sys
import time
import math
import json
import os
import signal
import struct

# --- Configuration ---
MAVLINK_PORT = int(os.environ.get("MAVLINK_PORT", "14540"))
CONNECTION_STRING = f"udp:127.0.0.1:{MAVLINK_PORT}"
MISSION_SPEED = 5.0          # m/s for auto missions
TAKEOFF_ALT_MIN = 2.5        # min takeoff altitude

# PX4 SITL default home position (gazebo x500)
DEFAULT_HOME_LAT = 47.397742
DEFAULT_HOME_LON = 8.545594
DEFAULT_HOME_ALT = 488.0     # AMSL

# --- Import pymavlink ---
try:
    from pymavlink import mavutil, mavwp
except ImportError:
    print("ERROR: pymavlink not found.")
    print("Install: pip install pymavlink")
    print("Or use PX4 venv: .venv/bin/python3 px4_mission_runner.py ...")
    sys.exit(1)


def connect_wait(connection_string, timeout=30):
    """Connect to PX4 and wait for heartbeat."""
    print(f"[CONN] Connecting to {connection_string}...")
    conn = mavutil.mavlink_connection(connection_string)

    print("[CONN] Waiting for heartbeat...")
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            mode_name = mavutil.mode_string_v10(msg)
            print(f"[CONN] Heartbeat from PX4 (type={msg.type}, "
                  f"autopilot={msg.autopilot}, mode={mode_name})")
            return conn
        print(f"       Still waiting... ({int(time.time() - start)}s)")
    raise TimeoutError(f"No heartbeat within {timeout}s")


def arm(conn, timeout=10):
    """Arm the drone."""
    print("[ARM] Arming...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm (1 = arm, 0 = disarm)
        0, 0, 0, 0, 0, 0
    )

    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("[ARM] Armed successfully")
            return
    raise TimeoutError("Arm failed")


def disarm(conn):
    """Disarm the drone."""
    print("[DISARM] Disarming...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)
    print("[DISARM] Disarmed")


def get_mode_id(conn, mode_name):
    """Get mode ID, trying exact match then substring match.
    Returns (actual_name, custom_mode_int)."""
    mapping = conn.mode_mapping()
    if mode_name in mapping:
        val = mapping[mode_name]
        # Handle both int and tuple (some pymavlink versions return tuple)
        if isinstance(val, tuple):
            if len(val) == 3:
                return mode_name, val[1]  # (base_mode, custom_mode, custom_sub_mode)
            return mode_name, val[0]
        return mode_name, val
    # Try case-insensitive
    for k, v in mapping.items():
        if k.upper() == mode_name.upper():
            if isinstance(v, tuple):
                if len(v) == 3:
                    return k, v[1]
                return k, v[0]
            return k, v
    # Try substring match (e.g. "AUTO.MISSION" matches "AUTO")
    for k, v in mapping.items():
        if mode_name.upper() in k.upper():
            if isinstance(v, tuple):
                if len(v) == 3:
                    return k, v[1]
                return k, v[0]
            return k, v
    return None, None


def set_mode(conn, mode_name, timeout=5):
    """Set flight mode with PX4-compatible fallback."""
    actual_name, mode_id = get_mode_id(conn, mode_name)
    if mode_id is None:
        # Try common PX4 custom_mode values
        px4_modes = {
            'AUTO': 4, 'OFFBOARD': 6, 'POSCTL': 4,
            'ALTCTL': 2, 'MANUAL': 0, 'STABILIZED': 1,
            'GUIDED': 4, 'LOITER': 4, 'MISSION': 4,
        }
        mode_id = px4_modes.get(mode_name.upper())
        actual_name = mode_name
        if mode_id is None:
            raise ValueError(f"Unknown mode: {mode_name} (available: {list(conn.mode_mapping().keys())})")

    print(f"[MODE] Switching to {mode_name} (custom_mode={mode_id})...")
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            current_mode = mavutil.mode_string_v10(msg)
            # Accept if current mode contains the target (e.g. "AUTO.MISSION" for "AUTO")
            if mode_name.upper() in current_mode.upper() or current_mode.upper() == mode_name.upper():
                print(f"[MODE] Mode confirmed: {current_mode}")
                return True
    print(f"[MODE] WARNING: Mode switch not confirmed (current: {current_mode if 'current_mode' in dir() else 'unknown'})")
    return False


def takeoff(conn, alt):
    """Takeoff to specified altitude (m)."""
    print(f"[TKOFF] Taking off to {alt}m...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0,  # pitch (ignored)
        0,  # empty
        0,  # empty
        float('nan'),  # yaw (nan = current)
        DEFAULT_HOME_LAT, DEFAULT_HOME_LON,  # lat/lon (ignored for takeoff)
        alt  # altitude
    )

    # Wait until reaching target altitude (±1m)
    print(f"[TKOFF] Waiting to reach {alt}m...")
    start = time.time()
    while time.time() - start < 30:
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            relative_alt = msg.relative_alt / 1000.0  # mm -> m
            if time.time() - start > 3:
                print(f"       Alt: {relative_alt:.1f}m")
            if relative_alt >= alt - 0.5:
                print(f"[TKOFF] Reached {relative_alt:.1f}m")
                return
    print("[TKOFF] WARNING: Takeoff timeout, proceeding anyway")


def land(conn):
    """Land the drone."""
    print("[LAND] Landing...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    # Wait until landed
    start = time.time()
    while time.time() - start < 60:
        msg = conn.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
        if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("[LAND] Landed")
            return
    print("[LAND] WARNING: Land timeout")


def wait_until_landed(conn, timeout=60):
    """Wait until the drone has landed."""
    print("[WAIT] Waiting for landed state...")
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
        if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("[WAIT] Landed confirmed")
            return True
    return False


# =============================================================================
# Mission Profiles
# =============================================================================

def mission_hover(conn, alt=5.0, duration=15.0):
    """Hover at current position for a duration."""
    takeoff(conn, alt)
    print(f"[HOVER] Hovering at {alt}m for {duration}s...")
    for _ in range(int(duration)):
        time.sleep(1)
        # Print position every 5s
        if int(duration - _) % 5 == 0:
            msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                print(f"       Alt: {msg.relative_alt/1000:.1f}m  "
                      f"Remaining: {int(duration - _)}s")
    return True


def mission_goto(conn, x, y, z, yaw=float('nan'), speed=MISSION_SPEED):
    """Fly to a local NED position (x,y,z in meters)."""
    takeoff(conn, max(abs(z), TAKEOFF_ALT_MIN))
    time.sleep(2)

    print(f"[GOTO] Flying to position x={x} y={y} z={z} (NED)...")

    # Switch to GUIDED mode for setpoint control
    mode_id = conn.mode_mapping().get('GUIDED')
    if mode_id is None:
        mode_id = conn.mode_mapping().get('OFFBOARD')
        if mode_id is None:
            # Fallback: use POSCTL
            mode_id = conn.mode_mapping().get('POSCTL')

    mode_name = [k for k, v in conn.mode_mapping().items() if v == mode_id][0]
    set_mode(conn, mode_name)

    # Send position target
    type_mask = 0b0000111111111000  # position only, ignore vel/accel/yaw

    if mode_name == 'OFFBOARD':
        # Also send setpoint to keep offboard alive
        for _ in range(30):
            conn.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x, y, z,  # position
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                yaw, 0    # yaw, yaw_rate
            )
            time.sleep(0.1)

    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        yaw, 0
    )

    # Monitor arrival
    print("[GOTO] Waiting to reach target...")
    start = time.time()
    while time.time() - start < 60:
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            dx = msg.x - x
            dy = msg.y - y
            dz = msg.z - z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if time.time() - start > 5:
                print(f"       NED: ({msg.x:.1f},{msg.y:.1f},{msg.z:.1f})  dist={dist:.1f}m")
            if dist < 1.5:
                print(f"[GOTO] Arrived at target (dist={dist:.1f}m)")
                return True
    print("[GOTO] WARNING: Goto timeout")
    return False


def mission_circle(conn, radius=10.0, alt=10.0, turns=1, speed=3.0):
    """Fly a circle pattern using DO_REPOSITION commands (works in LOITER/GUIDED)."""
    takeoff(conn, alt)
    time.sleep(2)

    print(f"[CIRCLE] Flying circle: radius={radius}m, alt={alt}m, turns={turns}")

    # Get current position as circle center
    current_pos = None
    for _ in range(5):
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_pos = (msg.lat / 1e7, msg.lon / 1e7)
            break

    if not current_pos:
        current_pos = (DEFAULT_HOME_LAT, DEFAULT_HOME_LON)

    center_lat, center_lon = current_pos
    print(f"[CIRCLE] Center: ({center_lat:.6f}, {center_lon:.6f})")

    # NED-to-GPS approximation: 1 deg lat ≈ 111320m, 1 deg lon ≈ 111320 * cos(lat)
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(center_lat))

    # Number of waypoints for one circle
    num_points = 24  # 15 degrees each
    total_points = num_points * turns

    print(f"[CIRCLE] {total_points} waypoints, radius={radius}m")

    for i in range(total_points):
        angle = 2 * math.pi * i / num_points  # radians
        # Offset in meters
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        # Convert to degrees
        dlat = dx / meters_per_deg_lat
        dlon = dy / meters_per_deg_lon
        target_lat = center_lat + dlat
        target_lon = center_lon + dlon

        # Send reposition command
        conn.mav.command_int_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 1,  # current, autocontinue
            speed,  # groundspeed m/s
            0,  # bitmask (0 = default)
            float('nan'), 0, 0,  # yaw, lat, lon
            int(target_lat * 1e7), int(target_lon * 1e7),
            alt  # altitude
        )

        # Wait for drone to approach the waypoint
        wait_time = (2 * math.pi * radius / num_points) / speed  # time to travel chord
        time.sleep(wait_time)

        # Print progress every 90 degrees
        if i % 6 == 0:
            progress = i * 100 / total_points
            pos = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if pos:
                print(f"       {progress:.0f}%  pos:({pos.lat/1e7:.5f},{pos.lon/1e7:.5f})  alt:{pos.relative_alt/1000:.1f}m")

    print(f"[CIRCLE] Circle complete!")
    return True


def mission_route(conn, profile_path):
    """Execute a multi-waypoint mission from a JSON profile file."""
    with open(profile_path) as f:
        profile = json.load(f)

    takeoff_alt = profile.get("takeoff_alt", TAKEOFF_ALT_MIN)
    waypoints = profile.get("waypoints", [])
    land_after = profile.get("land_after", True)

    if not waypoints:
        print("[ROUTE] No waypoints defined!")
        return False

    print(f"[ROUTE] Loading mission: {profile.get('name', 'unnamed')}")
    print(f"[ROUTE] {len(waypoints)} waypoints")

    takeoff(conn, takeoff_alt)
    time.sleep(2)

    # Build mission
    wp = mavwp.MAVWPLoader()
    wp.clear()
    seq = 0

    for wpt in waypoints:
        cmd_type = wpt.get("type", "waypoint").upper()
        lat = wpt.get("lat", DEFAULT_HOME_LAT)
        lon = wpt.get("lon", DEFAULT_HOME_LON)
        alt = wpt.get("alt", takeoff_alt)

        if cmd_type == "TAKEOFF":
            cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            param1 = wpt.get("pitch", 0)
            param4 = wpt.get("yaw", float('nan'))
        elif cmd_type == "LAND":
            cmd = mavutil.mavlink.MAV_CMD_NAV_LAND
            param1 = 0
        elif cmd_type == "LOITER":
            cmd = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
            param1 = wpt.get("duration", 10)  # seconds
            param3 = wpt.get("radius", 5)
        elif cmd_type == "CIRCLE":
            cmd = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
            param1 = wpt.get("turns", 2)
            param3 = wpt.get("radius", 10)
        elif cmd_type == "RTL":
            cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        else:  # WAYPOINT
            cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            param1 = wpt.get("hold_time", 0)
            param2 = wpt.get("accept_radius", 2)
            param3 = wpt.get("pass_radius", 0)
            param4 = wpt.get("yaw", float('nan'))

        wp.add(mavutil.mavlink.MAVLink_mission_item_int_message(
            conn.target_system, conn.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            cmd, 0, 1,
            wpt.get("param1", param1 if 'param1' in dir() else 0),
            wpt.get("param2", param2 if 'param2' in dir() else 0),
            wpt.get("param3", param3 if 'param3' in dir() else 0),
            param4 if 'param4' in dir() else 0,
            int(lat * 1e7), int(lon * 1e7),
            alt
        ))
        seq += 1

    # If land_after is true and no LAND waypoint exists, add one at the last position
    if land_after and waypoints[-1].get("type", "").upper() != "LAND":
        last = waypoints[-1]
        wp.add(mavutil.mavlink.MAVLink_mission_item_int_message(
            conn.target_system, conn.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
            0, 0, 0, 0,
            int(last.get("lat", DEFAULT_HOME_LAT) * 1e7),
            int(last.get("lon", DEFAULT_HOME_LON) * 1e7),
            0
        ))
        seq += 1

    # Upload mission
    print(f"[ROUTE] Uploading {wp.count()} waypoints...")
    conn.mav.mission_count_send(conn.target_system, conn.target_component, wp.count())

    for i in range(wp.count()):
        msg = conn.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'], blocking=True, timeout=10)
        if msg and msg.get_type() == 'MISSION_REQUEST':
            conn.mav.send(wp.wp(msg.seq))
            print(f"       WP {msg.seq + 1}/{wp.count()} uploaded")
        elif msg and msg.get_type() == 'MISSION_ACK':
            break

    # Wait for final ACK
    msg = conn.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if msg and getattr(msg, 'type', -1) == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("[ROUTE] Mission accepted")
    else:
        print("[ROUTE] Proceeding anyway (ack not confirmed)...")

    # Switch to AUTO mode with fallbacks
    ok = set_mode(conn, 'AUTO')
    if not ok:
        for fallback in ['AUTO.MISSION', 'MISSION', 'Auto']:
            try:
                if set_mode(conn, fallback):
                    ok = True
                    break
            except ValueError:
                continue
    if not ok:
        print("[ROUTE] ERROR: Cannot switch to AUTO mode, mission aborted")
        return False

    # Monitor
    print("[ROUTE] Executing mission...")
    last_seq = -1
    start = time.time()
    while time.time() - start < 300:  # 5 min max
        msg = conn.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1)
        if msg and msg.seq != last_seq:
            last_seq = msg.seq
            pos = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            alt_str = f" alt={pos.relative_alt/1000:.1f}m" if pos else ""
            print(f"       WP {msg.seq}/{wp.count()} reached{alt_str}")

        # Check if landed
        state = conn.recv_match(type='EXTENDED_SYS_STATE', blocking=False)
        if state and state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("[ROUTE] Landed, mission complete")
            break

    return True


# =============================================================================
# CLI
# =============================================================================

def print_usage():
    print("""Usage: px4_mission_runner.py <command> [args...]

Commands:
  hover [alt] [duration]       Hover at altitude (default 5m, 15s)
  goto  X Y Z [yaw]            Fly to local NED position
  circle [radius] [alt] [turns]  Fly a circle pattern
  route <profile.json>         Execute mission from JSON profile
  land                          Land immediately

Environment:
  MAVLINK_PORT    UDP port (default 14540 = SITL offboard)

Examples:
  python3 px4_mission_runner.py hover 10 30
  python3 px4_mission_runner.py goto 20 0 -10
  python3 px4_mission_runner.py circle 15 10 3
  python3 px4_mission_runner.py route ../missions/scan_area.json
""")


def main():
    if len(sys.argv) < 2:
        print_usage()
        sys.exit(1)

    command = sys.argv[1].lower()

    conn = connect_wait(CONNECTION_STRING)

    try:
        if command == "hover":
            alt = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
            arm(conn)
            mission_hover(conn, alt, duration)
            land(conn)

        elif command == "goto":
            if len(sys.argv) < 5:
                print("ERROR: goto requires X Y Z arguments")
                print_usage()
                sys.exit(1)
            x = float(sys.argv[2])
            y = float(sys.argv[3])
            z = float(sys.argv[4])
            yaw = float(sys.argv[5]) if len(sys.argv) > 5 else float('nan')
            arm(conn)
            mission_goto(conn, x, y, z, yaw)
            land(conn)

        elif command == "circle":
            radius = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
            alt = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
            turns = int(sys.argv[4]) if len(sys.argv) > 4 else 3
            arm(conn)
            mission_circle(conn, radius, alt, turns)
            time.sleep(2)
            land(conn)

        elif command == "route":
            if len(sys.argv) < 3:
                print("ERROR: route requires a JSON profile path")
                print_usage()
                sys.exit(1)
            arm(conn)
            mission_route(conn, sys.argv[2])
            # Route mission includes land via AUTO if land_after=true

        elif command == "land":
            land(conn)

        else:
            print(f"Unknown command: {command}")
            print_usage()
            sys.exit(1)

        # Final disarm
        time.sleep(3)
        disarm(conn)
        print("[DONE] Mission complete")

    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted")
        land(conn)
        disarm(conn)
    except Exception as e:
        print(f"\n[ERROR] {e}")
        try:
            land(conn)
            disarm(conn)
        except Exception:
            pass
        sys.exit(1)
    finally:
        conn.close()


if __name__ == "__main__":
    main()
