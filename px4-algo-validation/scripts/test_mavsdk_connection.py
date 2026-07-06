#!/usr/bin/env python3
"""MAVSDK 连接测试"""
import asyncio, sys
import mavsdk

async def main():
    drone = mavsdk.System()
    print("测试 MAVSDK 连接 127.0.0.1:14580...")
    await drone.connect(system_address="udp://127.0.0.1:14580")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ 已连接")
            break
        await asyncio.sleep(0.1)
    
    # 读取几个参数
    print("\n测试参数读取...")
    for pname in ["MC_INDI_ENABLE", "MC_ROLLRATE_P", "MC_PITCHRATE_P"]:
        try:
            v = await drone.param.get_parameter_float(pname)
            print(f"  {pname} = {v}")
        except Exception as e:
            print(f"  {pname}: {e}")
    
    # 测试遥测
    print("\n测试遥测数据流...")
    count = 0
    async for pos in drone.telemetry.position():
        print(f"  pos: alt={pos.relative_altitude_m:.2f}m lat={pos.latitude_deg:.6f}")
        count += 1
        if count >= 3: break
    
    print(f"\n✓ 连接测试成功 (收到 {count} 条遥测)")
    return True

asyncio.run(main())
