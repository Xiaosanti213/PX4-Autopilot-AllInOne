#!/usr/bin/env python3
"""
PX4 飞行日志分析脚本
用法: python3 log_analyzer.py <flight.ulg>
"""

import sys
import os
from pathlib import Path

try:
    from pyulog import ULog
    import pandas as pd
    import matplotlib.pyplot as plt
except ImportError as e:
    print(f"缺少依赖: {e}")
    print("安装: pip install --break-system-packages pyulog pandas matplotlib")
    sys.exit(1)

def analyze_flight(ulg_path: str):
    print(f"\n{'='*60}")
    print(f"分析日志: {ulg_path}")
    print(f"{'='*60}\n")
    
    # 加载日志
    ulog = ULog(ulg_path)
    
    # 基本信息
    print("[基本信息]")
    print(f"  时长: {ulog.last_timestamp - ulog.start_timestamp:.2f} 秒")
    print(f"  话题数: {len(ulog.data_list)}")
    
    # 列出关键话题
    topics = [d.name for d in ulog.data_list]
    key_topics = ['vehicle_attitude', 'vehicle_local_position', 
                  'vehicle_status', 'actuator_outputs']
    
    print("\n[关键话题]")
    for t in key_topics:
        if t in topics:
            print(f"  ✓ {t}")
        else:
            print(f"  ✗ {t} (未找到)")
    
    # 提取姿态数据
    if 'vehicle_attitude' in topics:
        print("\n[姿态分析]")
        att = ulog.get_dataset('vehicle_attitude').data
        df = pd.DataFrame(att)
        
        # 时间转换 (us -> s)
        t = df['timestamp'] / 1e6
        t = t - t.iloc[0]
        
        # 统计
        print(f"  Roll 速度: mean={df['rollspeed'].mean():.4f}, max={df['rollspeed'].max():.4f}")
        print(f"  Pitch 速度: mean={df['pitchspeed'].mean():.4f}, max={df['pitchspeed'].max():.4f}")
        print(f"  Yaw 速度: mean={df['yawspeed'].mean():.4f}, max={df['yawspeed'].max():.4f}")
        
        # 绘图
        output_dir = Path(ulg_path).parent.parent / 'data' / 'plots'
        output_dir.mkdir(parents=True, exist_ok=True)
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        
        axes[0].plot(t, df['rollspeed'], label='Roll rate', color='red')
        axes[0].set_ylabel('Roll rate (rad/s)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        
        axes[1].plot(t, df['pitchspeed'], label='Pitch rate', color='green')
        axes[1].set_ylabel('Pitch rate (rad/s)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        axes[2].plot(t, df['yawspeed'], label='Yaw rate', color='blue')
        axes[2].set_ylabel('Yaw rate (rad/s)')
        axes[2].set_xlabel('Time (s)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        
        plt.suptitle('Attitude Rates')
        plt.tight_layout()
        
        plot_path = output_dir / f"{Path(ulg_path).stem}_attitude.png"
        plt.savefig(plot_path, dpi=150)
        print(f"\n  图表已保存: {plot_path}")
    
    # 提取位置数据
    if 'vehicle_local_position' in topics:
        print("\n[位置分析]")
        pos = ulog.get_dataset('vehicle_local_position').data
        df = pd.DataFrame(pos)
        
        # 高度变化
        z = -df['z']  # NED 坐标系，z 向下为正
        print(f"  最大高度: {z.max():.2f} m")
        print(f"  平均高度: {z.mean():.2f} m")
        
        # XY 轨迹
        output_dir = Path(ulg_path).parent.parent / 'data' / 'plots'
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(df['x'], df['y'], 'b-', linewidth=1)
        ax.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', markersize=10, label='Start')
        ax.plot(df['x'].iloc[-1], df['y'].iloc[-1], 'ro', markersize=10, label='End')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Flight Trajectory (XY)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        plot_path = output_dir / f"{Path(ulg_path).stem}_trajectory.png"
        plt.savefig(plot_path, dpi=150)
        print(f"  轨迹图已保存: {plot_path}")
    
    print(f"\n{'='*60}")
    print("分析完成")
    print(f"{'='*60}\n")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 log_analyzer.py <flight.ulg>")
        print("\n查找最新日志:")
        log_dir = Path("/mnt/d/test/PX4-Autopilot/build/px4_sitl_default/rootfs/log")
        if log_dir.exists():
            ulg_files = sorted(log_dir.rglob("*.ulg"), key=lambda p: p.stat().st_mtime, reverse=True)
            if ulg_files:
                print(f"\n最新日志: {ulg_files[0]}")
        sys.exit(1)
    
    analyze_flight(sys.argv[1])
