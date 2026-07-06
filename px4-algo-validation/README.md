# PX4 算法验证工作流

## 目录说明

- `papers/` - 论文资料（PDF + 笔记）
- `code/` - 算法代码和 PX4 patch
- `scripts/` - 验证脚本（飞行测试、日志分析）
- `logs/` - 飞行日志（git-ignored）
- `data/` - 数据分析结果

## 快速开始

1. 启动 PX4 SITL:
   ```bash
   cd /mnt/d/test/PX4-Autopilot
   source Tools/simulation/gz/entrypoint.sh
   PX4_SIM_MODEL=gz_x500 HEADLESS=1 ./build/px4_sitl_default/bin/px4
   ```

2. 运行测试:
   ```bash
   python3 scripts/takeoff_hover_land.py
   ```

3. 分析日志:
   ```bash
   python3 scripts/log_analyzer.py logs/raw/session_001.ulg
   ```

## Git 工作流

每次算法修改 → 生成 patch → 测试 → 提交日志分析结果

```bash
# 修改 PX4 代码后
cd /mnt/d/test/PX4-Autopilot
git diff > /mnt/d/test/px4-algo-validation/code/patches/xxx.patch

# 提交工作记录
cd /mnt/d/test/px4-algo-validation
git add .
git commit -m "Test INDI controller with altitude 3m"
```
