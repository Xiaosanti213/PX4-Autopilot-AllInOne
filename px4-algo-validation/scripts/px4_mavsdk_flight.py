#!/usr/bin/env python3
"""PX4 SITL MAVSDK 飞行控制脚本"""
import asyncio, argparse, time, sys
from datetime import datetime

try:
    import mavsdk
    from mavsdk.offboard import OffboardError, VelocityNedYaw
except ImportError:
    print("mavsdk 未安装: pip install mavsdk")
    sys.exit(1)

class PX4FlightController:
    def __init__(self, controller="pid", host="127.0.0.1", port=14580):
        self.controller = controller
        self.host = host; self.port = port
        self.drone = mavsdk.System()
        self.altitude = 3.0; self.hover_time = 30
        self.connected = False
        
    async def connect(self, timeout=30):
        print(f"连接 PX4 SITL ({self.host}:{self.port})...")
        await self.drone.connect(system_address=f"udp://{self.host}:{self.port}")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.connected = True
                print(f"已连接 [{self.controller}]")
                return True
            await asyncio.sleep(0.1)
        return False
    
    async def set_params(self):
        """设置控制器参数"""
        if self.controller == "indi":
            print("配置 INDI 控制器...")
            try:
                await self.drone.param.set_parameter_float("MC_INDI_ENABLE", 1.0)
                await self.drone.param.set_parameter_float("MC_INDI_GAIN_P", 2.5)
                await self.drone.param.set_parameter_float("MC_INDI_GAIN_Y", 1.5)
                await self.drone.param.set_parameter_float("MC_INDI_FILTER", 0.5)
                print("  INDI 参数已设置")
            except Exception as e:
                print(f"  参数设置失败 (需 QGC): {e}")
        else:
            try:
                await self.drone.param.set_parameter_float("MC_INDI_ENABLE", 0.0)
                print("  PID 模式")
            except: pass
    
    async def check_health(self):
        print("检查状态...")
        async for h in self.drone.telemetry.health():
            print(f"  GPS:{'OK' if h.is_global_position_ok else 'NO'} "
                  f"IMU:{'OK' if h.is_gyrometer_calibration_ok else 'CAL'} "
                  f"EKF:{'OK' if h.is_local_position_ok else 'NO'}")
            return h.is_armable
    
    async def arm(self):
        print("解锁...")
        await self.drone.action.arm()
        print("已解锁")
    
    async def takeoff(self):
        print(f"起飞到 {self.altitude}m...")
        await self.drone.action.set_takeoff_altitude(self.altitude)
        await self.drone.action.takeoff()
        async for pos in self.drone.telemetry.position():
            if pos.relative_altitude_m >= self.altitude - 0.5:
                print(f"已达 {pos.relative_altitude_m:.1f}m")
                return True
            await asyncio.sleep(0.5)
        return True
    
    async def hover(self, duration=None):
        d = duration or self.hover_time
        print(f"悬停 {d}s...")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
        try: await self.drone.offboard.start()
        except: pass
        start = time.time()
        async for vel in self.drone.telemetry.velocity_ned():
            elapsed = time.time() - start
            if int(elapsed) % 5 == 0 and elapsed > 2:
                print(f"  {elapsed:.0f}s/{d}s vel(N={vel.north_m_s:.2f} E={vel.east_m_s:.2f} D={vel.down_m_s:.2f})")
            if elapsed >= d: print("悬停完成"); return True
            try: await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            except: pass
            await asyncio.sleep(0.1)
        return True
    
    async def land(self):
        print("降落...")
        await self.drone.action.land()
        async for pos in self.drone.telemetry.position():
            if pos.relative_altitude_m < 0.3:
                print("触地"); return True
            await asyncio.sleep(0.5)
        return True
    
    async def disarm(self):
        print("上锁...")
        try: await self.drone.action.disarm()
        except: pass
    
    async def run(self):
        print(f"\n{'='*50}\n  {self.controller.upper()} 飞行测试\n{'='*50}")
        if not await self.connect(): return False
        await self.set_params()
        if not await self.check_health(): return False
        await self.arm()
        await asyncio.sleep(1)
        if not await self.takeoff(): return False
        await asyncio.sleep(2)
        await self.hover()
        await self.land()
        await asyncio.sleep(5)
        await self.disarm()
        print(f"\n{'='*50}\n  {self.controller.upper()} 完成\n{'='*50}\n")
        return True

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("-c", "--controller", choices=["pid","indi"], default="pid")
    p.add_argument("--altitude", type=float, default=3.0)
    p.add_argument("--hover-time", type=int, default=30)
    args = p.parse_args()
    fc = PX4FlightController(controller=args.controller)
    fc.altitude = args.altitude; fc.hover_time = args.hover_time
    asyncio.run(fc.run())
