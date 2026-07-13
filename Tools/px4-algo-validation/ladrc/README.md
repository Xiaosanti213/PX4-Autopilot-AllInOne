# PX4 Algorithm Validation

PX4 算法验证工具集，包含 ULG 飞行日志分析和 SITL 自动飞行任务。

## 工具结构

```
px4-algo-validation/
├── README.md                        # 本文件
├── px4_ulg_analyzer/                # ULG 飞行日志分析器
│   ├── SKILL.md                     # 工具文档
│   ├── assets/                      # 报告模板
│   ├── references/                  # 指标定义 & 主题参考
│   └── scripts/                     # 核心脚本
│       ├── analyze.py               # 单文件分析 (text/markdown/json/ai)
│       ├── compare.py               # 双日志对比
│       ├── extract_timeseries.py     # 时间序列提取
│       ├── flight_analyzer.py       # 飞行性能分析引擎
│       ├── report_generator.py      # 报告生成器
│       └── ulog_parser.py           # ULog 二进制解析器 (纯 Python)
└── sitl_mission/                    # SITL 自动飞行任务
    ├── README.md                    # 完整文档
    ├── scripts/                     # 飞行脚本
    │   ├── px4_auto_mission.sh      # 悬停验证 (编译→起飞→降落)
    │   ├── px4_goto.sh              # 飞往指定 NED 坐标
    │   ├── px4_circle.sh            # 圆圈盘旋
    │   ├── px4_route.sh             # 执行 JSON 航线
    │   ├── px4_mission_runner.py    # MAVLink 任务引擎 (pymavlink)
    │   └── px4_send_cmd.sh          # 向运行中的仿真发令
    └── missions/                    # 示例航线
        ├── square_survey.json       # 方形航测
        ├── figure8.json             # 8 字航线
        └── recon_mission.json       # 多阶段侦察
```

## 工作流

```
sitl_mission (飞行)  →  ULG 日志  →  px4_ulg_analyzer (分析)
    ↓                                    ↓
 仿真飞行采集数据                    解析→对比→评分→报告
```

### 1. 飞行数据采集

```bash
# 在 WSL 中启动 PX4 SITL
cd Tools/px4-algo-validation/sitl_mission

# 悬停验证
bash scripts/px4_auto_mission.sh 5 15

# 圆圈盘旋
bash scripts/px4_circle.sh 10 5 3

# 自定义航线
bash scripts/px4_route.sh missions/square_survey.json
```

### 2. ULG 日志分析

```bash
cd Tools/px4-algo-validation/px4_ulg_analyzer

# AI 摘要
python scripts/analyze.py flight.ulg -f ai

# Markdown 完整报告
python scripts/analyze.py flight.ulg -f markdown -o report.md

# 双日志对比
python scripts/compare.py before.ulg after.ulg --labels "Before" "After"
```

## 依赖

- **px4_ulg_analyzer**: 零依赖，纯 Python 标准库
- **sitl_mission**: 需要 Python 3.6+ + pymavlink (`pip install pymavlink`)

## 控制器算法验证流程

```
1. 修改 PX4 源码 (如姿态控制器)
2. make px4_sitl gz_x500 编译
3. sitl_mission → 飞行采集 ULG
4. px4_ulg_analyzer → 分析性能指标
5. 对比不同算法 (INDI vs PID) 的评分
```
