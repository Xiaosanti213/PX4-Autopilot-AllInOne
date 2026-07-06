#!/usr/bin/env python3
"""
INDI vs PID 性能对比分析
用法: python3 compare_controllers.py --pid-log logs/raw/pid_hover.ulg --indi-log logs/raw/indi_hover.ulg
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

try:
    from pyulog import ULog
except ImportError:
    print("缺少 pyulog: pip install --break-system-packages pyulog")
    exit(1)

def load_attitude_data(ulg_path: str) -> pd.DataFrame:
    """加载姿态数据"""
    ulog = ULog(ulg_path)
    
    # 实际姿态
    att = ulog.get_dataset('vehicle_attitude').data
    df_att = pd.DataFrame(att)
    df_att['t'] = df_att['timestamp'] / 1e6  # us -> s
    df_att['t'] = df_att['t'] - df_att['t'].iloc[0]
    
    # 姿态设定值
    att_sp = ulog.get_dataset('vehicle_attitude_setpoint').data
    df_sp = pd.DataFrame(att_sp)
    df_sp['t'] = df_sp['timestamp'] / 1e6
    df_sp['t'] = df_sp['t'] - df_sp['t'].iloc[0]
    
    # 合并（按时间对齐）
    df = df_att[['t', 'roll', 'pitch', 'yaw']].copy()
    df['roll_sp'] = np.interp(df['t'], df_sp['t'], df_sp['roll_body'])
    df['pitch_sp'] = np.interp(df['t'], df_sp['t'], df_sp['pitch_body'])
    
    return df

def compute_metrics(df: pd.DataFrame) -> dict:
    """计算性能指标"""
    metrics = {}
    
    # 姿态跟踪误差 (RMSE)
    metrics['roll_rmse'] = np.sqrt(np.mean((df['roll'] - df['roll_sp'])**2))
    metrics['pitch_rmse'] = np.sqrt(np.mean((df['pitch'] - df['pitch_sp'])**2))
    
    # 姿态抖动（标准差）
    metrics['roll_std'] = df['roll'].std()
    metrics['pitch_std'] = df['pitch'].std()
    
    # 控制输出抖动（需要 actuator_outputs 话题）
    # metrics['ctrl_std'] = ...
    
    return metrics

def plot_comparison(df_pid: pd.DataFrame, df_indi: pd.DataFrame, output_path: str):
    """绘制对比图"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Roll 对比
    axes[0].plot(df_pid['t'], np.degrees(df_pid['roll']), 'r--', label='PID roll', alpha=0.7)
    axes[0].plot(df_pid['t'], np.degrees(df_pid['roll_sp']), 'r:', label='PID roll_sp', alpha=0.5)
    axes[0].plot(df_indi['t'], np.degrees(df_indi['roll']), 'b-', label='INDI roll', alpha=0.7)
    axes[0].plot(df_indi['t'], np.degrees(df_indi['roll_sp']), 'b:', label='INDI roll_sp', alpha=0.5)
    axes[0].set_ylabel('Roll (deg)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Pitch 对比
    axes[1].plot(df_pid['t'], np.degrees(df_pid['pitch']), 'r--', label='PID pitch', alpha=0.7)
    axes[1].plot(df_pid['t'], np.degrees(df_pid['pitch_sp']), 'r:', label='PID pitch_sp', alpha=0.5)
    axes[1].plot(df_indi['t'], np.degrees(df_indi['pitch']), 'b-', label='INDI pitch', alpha=0.7)
    axes[1].plot(df_indi['t'], np.degrees(df_indi['pitch_sp']), 'b:', label='INDI pitch_sp', alpha=0.5)
    axes[1].set_ylabel('Pitch (deg)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.suptitle('PID vs INDI Attitude Tracking')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"对比图已保存: {output_path}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pid-log', required=True, help='PID 控制器日志文件')
    parser.add_argument('--indi-log', required=True, help='INDI 控制器日志文件')
    args = parser.parse_args()
    
    print("\n=== 加载数据 ===")
    df_pid = load_attitude_data(args.pid_log)
    df_indi = load_attitude_data(args.indi_log)
    
    print("\n=== 计算指标 ===")
    metrics_pid = compute_metrics(df_pid)
    metrics_indi = compute_metrics(df_indi)
    
    print("\nPID 指标:")
    for k, v in metrics_pid.items():
        print(f"  {k}: {v:.6f}")
    
    print("\nINDI 指标:")
    for k, v in metrics_indi.items():
        print(f"  {k}: {v:.6f}")
    
    print("\n改进比例:")
    for k in metrics_pid:
        if k in metrics_indi:
            improvement = (metrics_pid[k] - metrics_indi[k]) / metrics_pid[k] * 100
            print(f"  {k}: {improvement:+.2f}%")
    
    print("\n=== 生成对比图 ===")
    output_dir = Path("/mnt/d/test/px4-algo-validation/data/plots")
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_path = output_dir / "pid_vs_indi_attitude.png"
    plot_comparison(df_pid, df_indi, str(plot_path))
    
    # 保存指标到 JSON
    result = {
        "pid": metrics_pid,
        "indi": metrics_indi,
        "improvement_percent": {k: (metrics_pid[k] - metrics_indi[k]) / metrics_pid[k] * 100 for k in metrics_pid}
    }
    
    json_path = Path("/mnt/d/test/px4-algo-validation/data/reports/comparison.json")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    import json
    with open(json_path, "w") as f:
        json.dump(result, f, indent=2)
    
    print(f"\n指标已保存: {json_path}")

if __name__ == "__main__":
    main()
