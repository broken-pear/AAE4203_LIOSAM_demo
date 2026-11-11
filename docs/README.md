# 文档索引

这个仓库做一件事：**在 Gazebo 里用同一份传感器数据、同一段轨迹，把 LOAM（A-LOAM 的
ROS 2 移植）和 LIO-SAM 各跑一遍，量化两者的精度差距，并说明差距在什么工况下变大。**

代码位置、怎么跑、结果在哪，下面三节说清楚。更细的内容在表格里的四份文档。

---

## 一分钟了解仓库结构

| 目录 | 是什么 | 从哪进 |
|---|---|---|
| `setup/` | 环境安装、两个上游算法的补丁、**7 套离线自检** | [`setup/install_deps.sh`](../setup/install_deps.sh) → `bash setup/check_all.sh` |
| `ros2_ws/src/lio_sam_gz_bringup/` | 仿真侧：机器人模型、世界、传感器预处理、脚本化驾驶员 | `launch/gazebo_lab1.launch.py` |
| `ros2_ws/src/lio_sam_bench/` | 实验侧：真值同步、轨迹录制、评测、画图、话题看门狗 | `launch/bench_replay.launch.py` |
| `experiments/` | 实验协议：录制 → 校验 → 双算法回放 → 评测 → 体检 → 汇总 | `bash experiments/run_bench.sh --scenarios s0_nominal` |
| `report/` | 英文 PPT 生成器（数字全部来自 CSV，不手打） | `python3 report/build_deck.py` |
| `results/` | 产物：`raw/` 原始轨迹、`tables/` 指标表、`figures/` 图、`manifest.json` 出处 | 生成后出现 |

## 三条命令

```bash
make check        # 7 套离线自检（不需要 ROS/Gazebo，约 50 秒）
make bench        # 采数据：6 场景 × 2 算法（约 35 分钟，已录的包会复用）
make deck         # 生成英文 PPT
```

`make` 只是把下面这些脚本收进一个入口，脚本本身没搬过，两种写法都有效：

```bash
bash setup/install_deps.sh                       # 一次性：装依赖 + clone 两个算法 + 编译
bash setup/check_all.sh                          # 任何时候：7 套离线自检
bash experiments/run_bench.sh --scenarios s0_nominal   # 采数据（录制 + 双算法 + 评测）
python3 report/build_deck.py                     # 出 PPT
```

`make`（不带参数）会列出全部目标：`deps check build demo record replay bench list deck clean`。

## 四份文档

| 文档 | 内容 | 什么时候看 |
|---|---|---|
| **[LIO-SAM-intro-and-roadmap.md](LIO-SAM-intro-and-roadmap.md)** | LIO-SAM 原理、因子图、与 LOAM 的结构差异、学习路线、参考文献 | 想搞懂两个算法**为什么**不一样 |
| **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** | 环境/编译/运行问题的逐条排查（A1~A6、B、C、D 分组） | **出问题时先查这份** |
| **[../experiments/README.md](../experiments/README.md)** | 实验协议：六个场景的设计意图、公平性约束、指标定义、时间戳关联规则 | 想复现或质疑数据 |
| **[../report/README.md](../report/README.md)** | PPT 怎么重新生成、阈值为什么这么定、渲染注意事项 | 改报告时 |

## 关键约定（读代码前先知道）

1. **公平性靠"同一个包"**：每个场景只录一次（`/points`、`/imu/data`、`/gt/odom`），
   两条算法各自回放同一份字节；不做任何重新仿真。
2. **真值来自仿真器**，不是任何一条算法的输出：`gz OdometryPublisher` → `/gt/odom`
   → 桥接 → 按点云时间戳插值（线性 + slerp），并按 `model.sdf` 补雷达杠杆臂。
3. **时间戳关联分两种模式**：LIO-SAM 的时间戳是精确的（`exact`），A-LOAM 只保留
   整数秒（`floor` + 全局帧滞后搜索）。评测器自动判别。
4. **报告里的每个数字都出自 `results/tables/all_runs.csv`**，PPT 生成器只读这个文件；
   数据不全时显示 `—`，绝不填充。
5. **数据先过体检**：`sanity_check.py` 会拦住"车没动""两条算法录到同一话题"
   "回放起点不一致"这类会让结论失效的情况。

## 当前状态

见根目录 [README](../README.md) 的"验证状态"表——那里逐项写明哪些已经实测通过、
哪些还待验证，以及每次实测的原始输出片段。
