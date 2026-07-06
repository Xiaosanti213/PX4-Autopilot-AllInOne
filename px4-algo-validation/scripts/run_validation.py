#!/usr/bin/env python3
"""
PX4 INDI Validation Workflow v3 - 带实时状态打印
"""
import asyncio, sys, argparse
from datetime import datetime

try:
    import mavsdk
except ImportError:
    print("ERROR: pip install mavsdk --break-system-packages"); sys.exit(1)

CONNECT_URL = "udpin://127.0.0.1:14540"

async def wait_for_connection(drone, timeout=60):
    print(f"[*] 等待连接 PX4 SITL (超时 {timeout}s)...")
    count = 0
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[+] 已连接 PX4!")
            return True
        count += 1
        if count >= timeout:
            print("[-] 连接超时!"); return False
        await asyncio.sleep(1)
    return False

async def get_current_telemetry(drone):
    """只抓取当前瞬间的状态，拿完立刻释放，绝不阻塞"""
    try:
        # 获取最新的一个飞行模式、位置和解锁状态数据
        fm = await drone.telemetry.flight_mode().__anext__()
        pos = await drone.telemetry.position().__anext__()
        armed_status = await drone.telemetry.armed().__anext__()

        mode = str(fm)
        z_str = f"alt={pos.relative_altitude_m:.2f}m" if pos else "alt=?"
        armed = "ARMED" if armed_status else "DISARMED"

        return f"{mode:12s} | {z_str} | {armed}"
    except Exception:
        return "TIMEOUT      | alt=? | UNKNOWN"

async def monitor_loop(drone, label, interval=1.0):
    """重新实现旧的 monitor_loop，内部调用上面的高效抓取函数"""
    status_line = await get_current_telemetry(drone)
    print(f"  [{label}] {status_line}")

async def set_indi_params(drone):
    print("\n[*] 设置 INDI 参数...")
    try:
        await drone.param.set_param_int("MC_INDI_ENABLE", 1)
        await asyncio.sleep(0.3)
        await drone.param.set_param_float("MC_INDI_GAIN_P", 2.5)
        await asyncio.sleep(0.3)
        await drone.param.set_param_float("MC_INDI_GAIN_Y", 1.5)
        await asyncio.sleep(0.3)
        await drone.param.set_param_float("MC_INDI_FILTER", 0.5)
        await asyncio.sleep(0.5)
        print("    MC_INDI_ENABLE=1, GAIN_P=2.5, GAIN_Y=1.5, FILTER=0.5")
    except Exception as e:
        print(f"    [WARN] 参数设置失败: {e}")
    return True

async def basic_flight_test(drone):
    print("\n=== 基础飞行测试 ===")
    # GPS
    print("[*] 等待 GPS 定位...")
    gps_ok = False
    for i in range(30):
        async for health in drone.telemetry.health_all_ok():
            gps_ok = bool(health)
            if gps_ok:
                print(f"[+] GPS 健康! (检查 {i+1}/30)")
            break
        if not gps_ok:
            await asyncio.sleep(1)
    if not gps_ok:
        async for pos in drone.telemetry.position():
            print(f"  当前位置: lat={pos.latitude_deg:.6f}, lon={pos.longitude_deg:.6f}, alt={pos.relative_altitude_m:.2f}m")
            break

    # Arm
    print("[*] 解锁...")
    try:
        await drone.action.arm()
        print("[+] 已解锁!")
    except Exception as e:
        print(f"[-] 解锁失败: {e}")
        async for cb in drone.telemetry.armed():
            print(f"    当前状态: {'ARMED' if cb else 'DISARMED'}"); break
        return

    # Takeoff
    print("[*] 起飞到 3m...")
    try:
        await drone.action.set_takeoff_altitude(3.0)
        await drone.action.takeoff()
        print("[+] takeoff() 已发送")
    except Exception as e:
        print(f"[-] 起飞命令失败: {e}")

    # Monitor for 20s
    print("\n  --- 实时状态监控 (20s) ---")
    for i in range(20):
        await monitor_loop(drone, f"{i+1:02d}/20")
        await asyncio.sleep(1)

    # Land
    print("[*] 降落...")
    try:
        await drone.action.land()
        print("[+] land() 已发送，等待 15s...")
        await asyncio.sleep(15)
        try: await drone.action.disarm(); print("[+] 已锁定!")
        except: pass
    except Exception as e:
        print(f"[-] 降落失败: {e}")
    print("[+] 基础飞行测试完成!")

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', choices=['basic','offboard','both'], default='basic')
    parser.add_argument('--controller', choices=['indi','pid'], default='pid')
    parser.add_argument('--url', default=CONNECT_URL)
    args = parser.parse_args()

    print(f"=== PX4 INDI 验证工作流 v3 ===")
    print(f"时间: {datetime.now():%Y-%m-%d %H:%M:%S}")
    print(f"控制器: {args.controller} | 测试: {args.test} | 连接: {args.url}")

    drone = mavsdk.System()
    await drone.connect(system_address=args.url)
    if not await wait_for_connection(drone):
        return 1

    if args.controller == 'indi':
        await set_indi_params(drone)

    if args.test in ('basic','both'):
        await basic_flight_test(drone)
    print(f"\n=== 验证完成 ===")
    return 0

if __name__ == "__main__":
    exit(asyncio.run(main()))
