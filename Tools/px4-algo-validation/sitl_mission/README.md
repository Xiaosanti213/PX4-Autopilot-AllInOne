---
name: px4-sitl-mission
description: "PX4 SITL (gz_x500) 自动任务技能。支持悬停、定点飞行(goto)、圆圈盘旋(circle)、多航点航线(route)等飞行任务。用于 WSL 中启动仿真、执行起飞测试、SITL 验证。触发词：PX4、SITL、仿真、起飞、悬停、航线、circle、route、commander、gz_x500、gazebo、无人机测试。"
agent_created: true
---

# PX4 SITL Mission Skill

自动化 PX4 SITL (Gazebo Harmonic / gz_x500) 仿真全流程。支持多种飞行任务类型，通过 JSON 配置文件可以自定义任意航线。

## 目录结构

```
px4-sitl-mission/
├── SKILL.md                       # 本文档
├── scripts/
│   ├── px4_auto_mission.sh        # 全自动悬停验证（编译→起飞→降落）
│   ├── px4_mission_runner.py      # Python MAVLink 任务引擎（核心）
│   ├── px4_goto.sh                # 飞往指定 NED 坐标
│   ├── px4_circle.sh              # 圆圈盘旋
│   ├── px4_route.sh               # 执行 JSON 航线任务
│   └── px4_send_cmd.sh            # 向运行中的仿真发命令
└── missions/
    ├── square_survey.json         # 方形航测
    ├── figure8.json               # 8 字航线
    └── recon_mission.json         # 多阶段侦察任务
```

## 前置条件

- WSL (Ubuntu) 已安装
- PX4 源码位于 `D:\source\PX4-Autopilot-AllInOne`（WSL: `/mnt/d/source/PX4-Autopilot-AllInOne`）
- tmux 已安装
- Python venv 已创建 + pymavlink 已安装（一次性的）：
  ```bash
  cd /mnt/d/source/PX4-Autopilot-AllInOne
  python3 -m venv .venv
  .venv/bin/pip install pymavlink
  ```

## 任务类型一览

| 任务 | 脚本 | 核心能力 |
|------|------|----------|
| **悬停验证** | `px4_auto_mission.sh` | 编译→启动→check→起飞→悬停→监控→降落 |
| **定点飞行** | `px4_goto.sh X Y Z` | 飞往指定 NED 坐标 |
| **圆圈盘旋** | `px4_circle.sh R Alt N` | 以半径 R 盘旋 N 圈 |
| **航线任务** | `px4_route.sh profile.json` | 执行 JSON 定义的多航点航线 |
| **快速发令** | `px4_send_cmd.sh "cmd"` | 向运行中的仿真发任意命令 |

---

## 一、基础悬停验证 (`px4_auto_mission.sh`)

全自动流程：编译 → 启动 SITL → commander check → 起飞 → 监控 → 降落。

```bash
bash scripts/px4_auto_mission.sh           # 起飞 5m，监控 15s
bash scripts/px4_auto_mission.sh 10        # 起飞 10m
bash scripts/px4_auto_mission.sh 5 30      # 起飞 5m，监控 30s
SKIP_BUILD=1 bash scripts/px4_auto_mission.sh  # 跳过编译
```

环境变量：`PX4_DIR`, `SKIP_BUILD`, `SESSION`, `READY_TIMEOUT`

---

## 二、定点飞行 (`px4_goto.sh`)

飞往指定 NED 坐标（相对于起飞点）。

```bash
bash scripts/px4_goto.sh 20 0 -10      # 前 20m，上升 10m
bash scripts/px4_goto.sh 10 5 -15      # 前 10m，右 5m，上升 15m
bash scripts/px4_goto.sh 20 0 -10 1.57 # 带偏航角 (1.57 rad = 90度)
```

- 自动起飞到目标高度，然后飞往目标点
- 到达后自动降落上锁
- 需要 PX4 SITL 已运行（先执行悬停脚本或手动启动）

---

## 三、圆圈盘旋 (`px4_circle.sh`)

以指定半径和高度飞圆形航线。

```bash
bash scripts/px4_circle.sh 15 10 3      # 半径 15m, 高度 10m, 3 圈
bash scripts/px4_circle.sh 20           # 半径 20m, 高度/圈数用默认值
bash scripts/px4_circle.sh 10 15 5      # 半径 10m, 高度 15m, 5 圈
```

- 使用 MAVLink LOITER_TURNS 命令，PX4 自主控制
- 需要 PX4 SITL 已运行
- 完成后自动降落

---

## 四、航线任务 (`px4_route.sh`)

执行 JSON 格式的多航点航线，支持任意组合的 waypoint / loiter / circle / rtl。

```bash
bash scripts/px4_route.sh missions/square_survey.json
bash scripts/px4_route.sh missions/figure8.json
bash scripts/px4_route.sh missions/recon_mission.json
```

### Mission Profile JSON 格式

```json
{
  "name": "任务名称",
  "description": "任务描述",
  "takeoff_alt": 10,
  "land_after": true,
  "waypoints": [
    { "type": "waypoint", "lat": 47.397742, "lon": 8.545594, "alt": 10 },
    { "type": "waypoint", "lat": 47.397900, "lon": 8.545700, "alt": 15, "hold_time": 3 },
    { "type": "loiter",   "lat": 47.397800, "lon": 8.545800, "alt": 15, "duration": 10, "radius": 15 },
    { "type": "circle",   "lat": 47.397600, "lon": 8.545600, "alt": 20, "radius": 20, "turns": 2 },
    { "type": "rtl",      "lat": 47.397742, "lon": 8.545594, "alt": 0 }
  ]
}
```

### 支持的航点类型

| type | 说明 | 额外参数 |
|------|------|----------|
| `waypoint` | 普通航点 | `hold_time` (悬停秒数), `accept_radius` (到达半径) |
| `loiter` | 定点盘旋 (按时间) | `duration` (秒), `radius` (米) |
| `circle` | 定点盘旋 (按圈数) | `turns` (圈数), `radius` (米) |
| `takeoff` | 起飞 | `pitch` (俯仰角) |
| `land` | 降落 | - |
| `rtl` | 返航 | - |

---

## 五、自定义航线开发指南

要创建新的航线，只需编写 JSON 文件放入 `missions/`：

1. 参考现有示例创建新 JSON
2. `lat`/`lon` 基于 SITL 默认 home: (47.397742, 8.545594)
   - 每 0.0001 度 ≈ 11m (lat) / 7m (lon)
3. 运行: `bash scripts/px4_route.sh missions/你的航线.json`

如需更复杂的控制（如动态路径、按速度飞行），可以：
- 直接调用 `scripts/px4_mission_runner.py` 传入自定义参数
- 修改 `px4_mission_runner.py` 中的任务函数
- 在 JSON 中组合多种航点类型实现复杂航线

---

## WorkBuddy 执行指南

当用户通过 WorkBuddy 发起任务时，按以下流程执行。

### 启动 SITL

```powershell
# 清理
wsl -d Ubuntu -- bash -c "pkill -f px4 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; tmux kill-session -t px4_mission 2>/dev/null; echo ok"

# 启动（如需编译先去编译）
wsl -d Ubuntu -- bash -c "tmux new-session -d -s px4_mission 'cd /mnt/d/source/PX4-Autopilot-AllInOne && HEADLESS=1 make px4_sitl gz_x500 2>/dev/null'; echo tmux_ok"

# 等待就绪（轮询 commander check，约需 30-60s）
wsl -d Ubuntu -- bash -c "tmux send-keys -t px4_mission 'commander check' Enter; sleep 3; tmux capture-pane -t px4_mission -p -S -20"
```

### 执行任务

```powershell
# 悬停验证
wsl -d Ubuntu -- bash -c "cd /mnt/d/source/PX4-Autopilot-AllInOne && bash ~/.workbuddy/skills/px4-sitl-mission/scripts/px4_auto_mission.sh 5 15"

# 或分步执行（PX4 已运行时）
wsl -d Ubuntu -- bash -c "cd /mnt/d/source/PX4-Autopilot-AllInOne && .venv/bin/python3 /mnt/c/Users/Administrator/.workbuddy/skills/px4-sitl-mission/scripts/px4_mission_runner.py hover 10 20"

wsl -d Ubuntu -- bash -c "cd /mnt/d/source/PX4-Autopilot-AllInOne && .venv/bin/python3 /mnt/c/Users/Administrator/.workbuddy/skills/px4-sitl-mission/scripts/px4_mission_runner.py circle 15 10 3"

wsl -d Ubuntu -- bash -c "cd /mnt/d/source/PX4-Autopilot-AllInOne && .venv/bin/python3 /mnt/c/Users/Administrator/.workbuddy/skills/px4-sitl-mission/scripts/px4_mission_runner.py route /mnt/c/Users/Administrator/.workbuddy/skills/px4-sitl-mission/missions/square_survey.json"
```

### 清理

```powershell
wsl -d Ubuntu -- bash -c "tmux send-keys -t px4_mission C-c; sleep 2; tmux kill-session -t px4_mission 2>/dev/null; pkill -f px4 2>/dev/null; echo cleaned"
```

---

## 关键命令速查

| 命令 | 说明 |
|------|------|
| `commander check` | 预检状态 |
| `commander arm` / `commander disarm` | 解锁 / 上锁 |
| `commander takeoff` | 起飞到 MIS_TAKEOFF_ALT |
| `commander land` | 着陆 |
| `commander mode posctl` | 切换到位置控制模式 |
| `listener vehicle_local_position` | 查看位置/速度 |
| `listener sensor_mag` | 查看磁力计 |
| `param set NAME VALUE` | 设置参数 |
| `param show NAME` | 查看参数 |

## MAVLink 端口

| 端口 | 用途 |
|------|------|
| 14540 | SITL offboard (mission runner 默认) |
| 14550 | SITL GCS (QGC 等地面站) |
| 14580 | SITL onboard (板载计算机) |

## 注意事项

- **Python venv**: 需要在 PX4 目录下创建 `.venv` 并安装 pymavlink（一次性）
- **沙箱**: 所有 WSL 命令需要 `dangerouslyDisableSandbox: true`
- **NED 坐标系**: z 为负 = 上升
- **磁力计 STALE**: 如出现此问题，检查 `GZBridge.cpp` 中不应有 `* 1e-4` 额外转换
- **tmux 会话**: 默认为 `px4_mission`
- **MAVLink 端口**: Mission runner 连接 UDP 14540，需确保 PX4 已启动
