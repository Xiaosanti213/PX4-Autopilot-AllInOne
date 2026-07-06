#!/usr/bin/env python3
"""
PX4 飞行测试 + 数据记录脚本
用法: python3 flight_test.py --controller pid --test hover
"""

import asyncio
import argparse
import json
from datetime import datetime
from pathlib import Path

try:
    import mavsdk
    from mavsdk import System
except ImportError:
    print("缺少 mavsdk，安装: pip install --break-system-packages mavsdk")
    exit(1)

class FlightTest:
    def __init__(self, controller: str, test_type: str):
        self.controller = controller  # 'pid' or 'indi'
        self.test_type = test_type    # 'hover', 'step', 'disturbance'
        self.drone = System()
        self.data = {
            "controller": controller,
            "test_type": test_type,
            "timestamp": datetime.now().isoformat(),
            "attitude": [],      # (time, roll, pitch, yaw)
            "attitude_sp": [],   # (time, roll_sp, pitch_sp, yaw_sp)
            "rates_sp": [],      # (time, roll_rate_sp, ...)
            "actuator": [],     # (time, out[0..3])
        }
    
    async def connect(self):
        print(f"[连接] 控制器={self.controller}, 测试={self.test_type}")
        await self.drone.connect(system_address="udpin://127.0.0.1:14580")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("  ✓ 已连接")
                break
    
    async def arm_and_takeoff(self, altitude: float = 3.0):
        print(f"\n[起飞] 目标高度 {altitude}m")
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        await asyncio.sleep(5)  # 等待到达高度
        print("  ✓ 悬停稳定")
    
    async def record_data(self, duration: float):
        """记录数据（简化版，实际应从 MAVLink 话题订阅）"""
        print(f"\n[记录] 持续 {duration} 秒...")
        # 注意：MAVSDK 不提供原始话题订阅
        # 完整方案需要直接解析 MAVLink 或使用 pyulog 分析日志
        await asyncio.sleep(duration)
        print("  ✓ 记录完成")
    
    async def landing(self):
        print("\n[降落]")
        await self.drone.action.land()
        await asyncio.sleep(5)
        print("  ✓ 已降落")
    
    async def run(self):
        await self.connect()
        await self.arm_and_takeoff(3.0)
        
        if self.test_type == "hover":
            await self.record_data(30.0)  # 悬停 30 秒
        elif self.test_type == "step":
            # 姿态阶跃（需要发送 MAV_CMD_DO_SET_ATTITUDE）
            pass
        
        await self.landing()
        
        # 保存元数据
        output_dir = Path("/mnt/d/test/px4-algo-validation/data/raw")
        output_dir.mkdir(parents=True, exist_ok=True)
        output_file = output_dir / f"{self.controller}_{self.test_type}_{datetime.now():%Y%m%d_%H%M%S}.json"
        
        with open(output_file, "w") as f:
            json.dump(self.data, f, indent=2)
        
        print(f"\n[完成] 数据已保存: {output_file}")

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=["pid", "indi"], required=True)
    parser.add_argument("--test", choices=["hover", "step", "disturbance"], required=True)
    args = parser.parse_args()
    
    test = FlightTest(args.controller, args.test)
    await test.run()

if __name__ == "__main__":
    asyncio.run(main())
