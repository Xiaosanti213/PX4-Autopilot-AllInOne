# PX4 INDI 控制算法实现文档

生成时间: 2026-06-28 19:09:46

## 1. 概述

INDI (Incremental Nonlinear Dynamic Inversion) 是一种用于多旋翼飞行器速率控制的先进控制算法。
它通过直接利用 IMU 测量的角加速度来抵消飞行器动力学中的非线性项，实现更好的扰动抑制和跟踪性能。

**核心优势：**
- 直接使用 IMU 角加速度反馈（扰动敏感度高）
- 基于物理模型（刚体动力学的增量逆）
- 无需精确的模型参数
- 改进了传统 PID 在高机动性场景下的性能

## 2. 数学原理

### 2.1 刚体动力学

多旋翼飞行器的角动力学方程：

    M(ω)ω̇ + ω × M(ω)ω = τ

其中：
- ω = [p, q, r]ᵀ 是角速度向量（滚转/俯仰/偏航）
- M(ω) 是惯性矩阵（对称正定）
- τ = [τ_p, τ_q, τ_r]ᵀ 是控制力矩（电机差分产生）

### 2.2 INDI 核心公式

INDI 的核心思想是"增量逆"——仅逆推当前时刻的摄动，而不是完整模型：

    alpha_des = K_p × (omega_sp - omega)           [期望角加速度，P 控制]
    alpha_obs = LPF(alpha_imu)                      [观测角加速度，低通滤波]
    alpha_ff  = (omega_sp - omega_prev) / dt         [前馈加速度]

    u = (alpha_des - alpha_obs + alpha_ff) / g      [控制量]

其中：
- K_p = [K_roll, K_pitch, K_yaw]ᵀ 是 INDI 增益向量
- alpha_imu 是 IMU 测量的原始角加速度
- LPF() 是一阶低通滤波器（减小 IMU 噪声影响）
- g 是控制效能（电机差分产生单位力矩对应的控制量）
- alpha_ff 是前馈项（来自期望角速度变化率）

### 2.3 与 PID 的对比

| 特性 | PID | INDI |
|------|-----|------|
| 反馈信号 | 角速度误差积分 | 角加速度观测 |
| 扰动抑制 | 间接（通过积分） | 直接（加速度反馈） |
| 参数数量 | 3 (Kp, Ki, Kd) | 2-3 (Kp_roll, Kp_yaw, filter) |
| 计算复杂度 | 中等 | 较低 |
| 对模型依赖 | 无 | 极低（只需控制效能符号） |

## 3. 代码实现

### 3.1 参数定义 (mc_rate_control_params.yaml)

```yaml
MC_INDI_ENABLE:
  type: boolean
  default: 0        # 0=PID, 1=INDI
  description: 启用 INDI 速率控制器

MC_INDI_GAIN_P:
  type: float
  default: 2.5      # 滚转/俯仰轴 INDI P 增益
  min: 0.1, max: 10.0

MC_INDI_GAIN_Y:
  type: float
  default: 1.5      # 偏航轴 INDI P 增益
  min: 0.1, max: 10.0

MC_INDI_FILTER:
  type: float
  default: 0.5      # 低通滤波系数 alpha
  min: 0.1, max: 0.9
  description: 角加速度低通滤波系数，0.1=强滤波(噪声小但响应慢)，0.9=弱滤波(响应快但噪声大)
```

### 3.2 状态变量 (MulticopterRateControl.hpp)

```cpp
// INDI 状态变量
matrix::Vector3f _indi_rates_prev;    // 上一时刻角速度（前馈计算用）
AlphaFilter<float> _indi_alpha_filter; // 角加速度低通滤波器
```

### 3.3 初始化 (parameters_updated)

```cpp
// INDI 参数更新
_indi_alpha_filter.setParameters(_param_mc_indi_filter.get());
_indi_rates_prev.zero();
```

### 3.4 INDI 控制律 (Run)

```cpp
if (_param_mc_indi_enable.get()) {
    // 1. 低通滤波后的角加速度（减少 IMU 噪声）
    const Vector3f alpha_filtered{
        _indi_alpha_filter.update(angular_accel(0)),
        _indi_alpha_filter.update(angular_accel(1)),
        _indi_alpha_filter.update(angular_accel(2))
    };

    // 2. 角速度误差
    const Vector3f rate_error = _rates_setpoint - rates;

    // 3. 期望角加速度（K_p × rate_error）
    const Vector3f K_indi{
        _param_mc_indi_gain_p.get(),  // 滚转
        _param_mc_indi_gain_p.get(),  // 俯仰
        _param_mc_indi_gain_y.get()   // 偏航
    };
    const Vector3f alpha_des = rate_error.emult(K_indi);

    // 4. 前馈加速度（来自期望角速度变化率）
    const Vector3f alpha_ff = (_rates_setpoint - _indi_rates_prev)
                              / math::max(dt, 0.0001f);

    // 5. INDI 控制律
    torque_setpoint = (alpha_des - alpha_filtered + alpha_ff)
                      .emult(inv_ctrl_effectiveness);

    // 6. 限幅（防止电机饱和）
    for (int axis = 0; axis < 3; axis++) {
        torque_setpoint(axis) = math::constrain(torque_setpoint(axis), -1.0f, 1.0f);
    }

    // 7. 保存当前角速度（前馈用）
    _indi_rates_prev = rates;
} else {
    // 标准 PID 控制器
    torque_setpoint = _rate_control.update(rates, _rates_setpoint,
                                            angular_accel, dt,
                                            _maybe_landed || _landed);
}
```

## 4. 参数调优指南

### 4.1 调参步骤

1. **从 PID 切换到 INDI**：将 `MC_INDI_ENABLE` 设为 1
2. **设置初始增益**：MC_INDI_GAIN_P=2.5, MC_INDI_GAIN_Y=1.5
3. **飞行测试**：
   - 做快速滚转/俯仰指令，检查响应是否跟手
   - 观察是否有高频振荡（增益过高）或响应迟缓（增益过低）
4. **调整增益**：
   - 振荡 → 减小 MC_INDI_GAIN_P
   - 响应迟缓 → 增加 MC_INDI_GAIN_P（但 < 10）
5. **调整滤波**：默认 0.5 适合大多数情况
   - 高速机动场景 → 降低到 0.3（更平滑）
   - 精细控制场景 → 提高到 0.7（更响应）

### 4.2 典型配置

| 场景 | MC_INDI_GAIN_P | MC_INDI_GAIN_Y | MC_INDI_FILTER |
|------|---------------|---------------|----------------|
| 一般飞行 | 2.5 | 1.5 | 0.5 |
| 高速机动 | 4.0 | 2.0 | 0.3 |
| 精细悬停 | 1.5 | 1.0 | 0.7 |
| 有扰动风 | 3.0 | 2.0 | 0.4 |

## 5. 验证方案

### 5.1 飞行测试流程

```
阶段 1: 地面检查
  - 检查参数是否正确加载
  - 解锁后观察电机响应是否平滑

阶段 2: 悬停测试
  - 起飞到 3m 高度
  - 悬停 30 秒，观察位置漂移

阶段 3: 机动测试
  - 快速滚转 45°，记录响应时间和超调
  - 快速俯仰 45°，记录响应时间和超调
  - 偏航 180°，检查是否稳定

阶段 4: 扰动测试
  - 施加侧风扰动（手持气流）
  - 观察扰动后恢复时间

阶段 5: 定量对比（INDI vs PID）
  - 相同飞行轨迹，对比：
    - 跟踪误差 RMS
    - 最大超调量
    - 扰动恢复时间
    - 能耗（积分力矩）
```

### 5.2 日志分析方法

使用 `scripts/log_analyzer.py` 分析 `.ulg` 日志：

```bash
python3 log_analyzer.py <logfile.ulg>
```

关键指标：
- `rate_sp - rate` 的 RMS 值 → 跟踪精度
- `torque_setpoint` 的方差 → 控制能耗
- 最大 `angular_accel` → 加速度峰值

## 6. 文件清单

| 文件 | 修改内容 |
|------|---------|
| `mc_rate_control_params.yaml` | 新增 4 个 INDI 参数 |
| `MulticopterRateControl.hpp` | 新增 2 个 INDI 状态变量 + 4 个参数声明 |
| `MulticopterRateControl.cpp` | 新增 INDI 控制律实现 |
| `run_validation.py` | 自动化验证工作流脚本 |

## 7. 参考资料

- S. Baekert et al., "INDI (Incremental Nonlinear Dynamic Inversion) for Attitude Control"
- M. Kadri et al., "Experimental Validation of INDI for UAV Flight Control"
- PX4 Firmware: `src/modules/mc_rate_control/`

## 8. 待办事项

- [ ] SITL 仿真验证
- [ ] 实际飞行测试
- [ ] 参数整定优化
- [ ] 对比 PID 的量化性能指标
- [ ] 偏航 INDI 控制效能补偿（与滚转/俯仰不同）
- [ ] 多旋翼构型适配（X4/X6 等）
