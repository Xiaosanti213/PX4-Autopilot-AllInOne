#!/usr/bin/env python3
"""
PX4 SITL takeoff-hold-land script via MAVSDK
Requires: mavsdk (pip install mavsdk)
Usage: python3 px4_takeoff_hover_land.py
"""

import asyncio
import mavsdk
from mavsdk.offboard import PositionNedYaw
import time

async def run():
    print("[1] Connecting to PX4 SITL...")
    
    drone = mavsdk.System()
    # PX4 SITL exposes MAVLink on UDP 14540 by default
    await drone.connect(system_address="udpin://127.0.0.1:14580")
    
    print("[2] Waiting for heartbeat...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"    Connected!")
            break

    print("[3] Arming...")
    await drone.action.arm()
    print("    Armed!")

    print("[4] Taking off to 3m...")
    await drone.action.set_takeoff_altitude(3.0)
    await drone.action.takeoff()
    
    # Wait for takeoff to complete
    print("    Waiting for altitude...")
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= 2.8:
            print(f"    Reached {position.relative_altitude_m:.1f}m, holding...")
            break

    print("[5] Holding for 5 seconds...")
    await asyncio.sleep(5)

    print("[6] Landing...")
    await drone.action.land()
    print("    Landing initiated, waiting for touchdown...")

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("    Touchdown confirmed.")
            break

    print("\n[OK] Mission complete!")
    print(f"    Flight logs saved in: build/px4_sitl_default/rootfs/log/")

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"\n[ERROR] {type(e).__name__}: {e}")
        print("Is PX4 SITL running? Check: wsl -e bash -c 'ps aux | grep px4 | grep -v grep'")
