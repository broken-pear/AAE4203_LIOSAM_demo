# 公平对比实验：LOAM (baseline) vs LIO-SAM

> 这份文档是**实验协议**：怎么跑、为什么这么跑、结果怎么算。
> 报告（PPT）里的每一个数字都应该能在这份协议下复现。

---

## 0. 一句话

同一个 Gazebo 场景、同一份传感器数据、同一套评测器，只换算法。

```
Gazebo(16 线雷达 + 200 Hz IMU)
        │  录一次包（唯一数据源）
        ▼
experiments/data/<scenario>/  ──► check_bag.py：话题/条数/时长不合格就当场拦下
        │
        ├──► ros2 bag play ──► A-LOAM   ──► results/raw/<scenario>/loam.csv
        └──► ros2 bag play ──► LIO-SAM  ──► results/raw/<scenario>/liosam.csv
                                    │
                        evaluate.py ┴──► results/tables/*.md + *.csv + results/json/*.json
                                    └──► make_figures.py ──► results/figures/*.png
                                    └──► report/build_deck.py ──► PPT（数字自动填）
```

> `check_bag.py` 存在的理由：录制"成功"不等于数据可用。最坑的一种是 `/gt/odom`
> 一条消息都没有 —— bag 照写、两条算法照跑，直到评测才报 "gt 为空"，
> 那时已经浪费两轮回放。所以录完 10 秒内就验，并把根因写清楚。

**关键点**：包只录一次，两条算法都从这一份包回放。
所以两条算法看到的点云流逐字节相同、对齐到的真值逐帧相同 ——
结果差异只能来自算法本身。这是消融实验的标准做法，也是这份报告唯一站得住的比较方式。

---

## 1. 三条命令跑完

```bash
# 0) 每次开新终端（或写进 ~/.bashrc）
source /opt/ros/jazzy/setup.bash
source ~/AAE4203_LIOSAM_demo/ros2_ws/install/setup.bash
cd ~/AAE4203_LIOSAM_demo

# 1) 看有哪些场景
bash experiments/run_bench.sh --list

# 2) 先跑一组最快的验证链路（约 4 分钟）
bash experiments/run_bench.sh --scenarios s0_nominal

# 3) 正式跑（S0–S4，约 35~45 分钟；加 S5 再多 10 分钟）
bash experiments/run_bench.sh
```

跑完看这三样：

```bash
cat results/tables/all_runs.md          # 主表（报告/PPT 就用这张）
ls  results/figures/                     # 轨迹图、误差图、ATE 柱状图
cat results/json/s0_nominal.json         # 单个场景的全部原始指标
```

只跑 baseline、或只跑 SOTA：

```bash
bash experiments/run_bench.sh --algo loam
bash experiments/run_bench.sh --algo liosam
```

复用已录好的包（不再开 Gazebo，最省时间）：

```bash
bash experiments/run_bench.sh --replay-only --scenarios s0_nominal s3_corridor
```

---

## 2. 公平性协议（报告第 3 节直接引用这张表）

| 维度 | 两条算法完全一致的设置 |
|---|---|
| 场景几何 | 同一个 `.sdf`（`lab1_room` / `corridor` / `hall_sparse`），重力 9.80511 |
| 传感器 | 16 线 ±15°、360 列、10 Hz、量程 0.2–40 m、σ=0.01 m；IMU 200 Hz |
| 传感器数据 | **同一份 bag**（同一份点云、同一份 IMU、同样被插值的真值） |
| 轨迹 | 同一个 `scripted_driver` 脚本（开环、分段常量速度、同一 spawn 位姿） |
| 输入话题 | 两条算法都读 `/points`；LIO-SAM 额外读 `/imu/data`（LOAM 按定义不用 IMU） |
| 降采样 | LOAM 建图 0.1/0.2 m ＝ LIO-SAM `mappingCornerLeafSize`/`mappingSurfLeafSize` |
| 最近量程 | LOAM `minimum_range=0.3` ＝ LIO-SAM `lidarMinRange=0.3` |
| 回环 | 两边都**关闭**（LIO-SAM `loopClosureEnableFlag=false`；A-LOAM 无回环） |
| 评测 | 同一个 `evaluate.py`、同一种 SE(3) 对齐、同一组指标定义 |
| 真值 | 同一个 `OdometryPublisher` 插件输出的世界系位姿，插值到点云时间戳 |

**唯一的差异**是算法本身：LOAM 用 scan-to-scan + scan-to-map（纯几何、无去畸变、无 IMU、
用高斯-牛顿/Ceres 迭代），LIO-SAM 用 IMU 预积分 + 局部子图 scan-to-map + iSAM2 因子图。

---

## 3. 场景矩阵

| ID | 场景 | 世界 | 动作 | 时长 | 想暴露什么 |
|---|---|---|---|---|---|
| **S0** | 名义工况 | `lab1_room` 12×12 m | r=1.25 m 圆，v=0.40，ω=0.32 | 90 s | 基线：两者都该收敛 |
| **S1** | 高角速度 | `lab1_room` | r=0.5 m 圆，v=0.45，**ω=0.90** | 60 s | 运动畸变（每帧转 5.2°），去畸变的价值 |
| **S2** | 高线速度 | `lab1_room` | 1.2 m/s 方框 + 直角原地转 | 60 s | 帧间位移 0.12 m，初值好坏决定配准 |
| **S3** | 几何退化 | `corridor` 30×3 m | 沿轴直行 7.2 m + 掉头，2 来回 | 56 s | 沿轴方向 / yaw 由谁提供 |
| **S4** | 特征稀疏 | `hall_sparse` 20×20 m | r=3 m 圆，v=0.6，ω=0.20 | 60 s | 特征数量不足时谁更稳 |
| **S5** | 长时运行 | `lab1_room` | S0 的动作跑 180 s（9 圈） | 180 s | 累积漂移（两边都无回环） |

> S1/S2 的角速度、线速度都是算过的：0.1 s 扫一帧，
> ω=0.90 → 0.09 rad ≈ 5.2°；v=1.2 → 0.12 m。这两个量级正好让畸变从"可以忽略"变成"主导误差"。

---

## 4. 指标定义

| 指标 | 定义 | 为什么报告里要有它 |
|---|---|---|
| **ATE RMSE / mean / max** | SE(3) Umeyama 对齐后逐帧平移误差 | 主指标，和公开论文可比 |
| **ATE rot** | 对齐后姿态角误差 | 说明"位置对但朝向错"的情况 |
| **Raw RMSE** | 对齐前的逐帧误差 | 反映起始锚定后的整体偏差 |
| **RPE trans (%/m)** | 相隔 1 s 的相对平移误差 / 位移 | 局部精度，与尺度无关 |
| **RPE rot (deg/m)** | 相对旋转误差 / 位移 | 走廊/稀疏场景最敏感 |
| **Final pos / yaw err** | 对齐后最后一帧的误差 | 对应"绕回起点时接缝有多大" |
| **Frames > 0.5 m** | 误差超半米的帧数占比 | 鲁棒性；发散不是"平均误差大"，而是"某段彻底错了" |
| **Diverged** | 最大误差 > 5 m | 直接标记跑飞 |
| **Rate (Hz)** | 位姿输出频率 | 高频输出是 LIO-SAM 的工程优势之一 |

### 时间戳与关联（**这一节最容易出错，必须看**）

两条算法给的时间戳精度不一样：

| | 时间戳 | 关联方式 |
|---|---|---|
| LIO-SAM | 与点云一致（纳秒） | `exact`：最近邻匹配，容差 5 ms |
| A-LOAM | **只有整数秒**（上游把 `header.stamp.sec` 当 double 用，纳秒丢了） | `floor`：顺序配对 + 全局滞后搜索 + `floor()` 一致性校验 |

`evaluate.py` 会先看 95% 以上的 est 时间戳是不是整数秒，自动选模式，并把
`match_fraction` / `lag` 打到终端。**`match_fraction` 低于 0.9 就别看 ATE 了**，
先查话题名和 bag。

> 这也是为什么必须有 `gt_sync.py`：真值被重采样到**点云时间戳**上。
> 如果直接拿 A-LOAM 的整秒时间戳去找"最接近的真值"，在 0.4 m/s 下会引入最多 4 cm 的虚假误差 ——
> 那个误差是时间戳造成的，不是算法造成的。

---

## 5. 目录与产物

```
experiments/
├── run_bench.sh            主脚本（录包 → 回放 → 评测 → 汇总）
├── scenarios/              每个场景一份 yaml：动作脚本 + 元信息
│   ├── s0_nominal.yaml     ...  s5_long_run.yaml
└── data/                   ★ 录好的 bag（每个几百 MB，已 gitignore）
    └── s0_nominal/         metadata.yaml + *.db3

results/
├── raw/<scenario>/<algo>.csv   逐帧「算法位姿 + 真值」+ 同名 .meta.json
├── json/<scenario>.json        全部标量指标（可追溯）
├── tables/<scenario>.md/.csv   每个场景一张表 + 汇总 all_runs.{md,csv}
├── figures/                    轨迹图、误差图、ATE 柱状图（PNG 160 dpi）
└── logs/                       每个进程的完整终端输出（出问题时先看这里）
```

`results/raw/<algo>.csv` 的列固定：

```
kind,stamp,node_now,x,y,z,qx,qy,qz,qw
  gt   真值，stamp = 点云时间戳
  est  算法位姿，stamp = 算法自己的时间戳
  node_now = 收到消息时的仿真时刻
```

---

## 6. 出问题先看这里

| 现象 | 原因 | 处理 |
|---|---|---|
| `record` 阶段没有 `/points` | 桥接/传感器没起来 | 看 `results/logs/record_*.log`；先单跑 `gazebo_lab1.launch.py` 确认 |
| bag 录到了但 `/gt/odom` 是空的 | `model.sdf` 的 OdometryPublisher 没加载 / 桥接没配 | `ros2 topic hz /gt/odom`，应为 100 Hz |
| 回放后 CSV 里 `est` 只有几行 | 算法没起来 / 话题名变了 | 看 `results/logs/replay_*.log`；`ros2 node list` 应能看到 3~4 个算法节点 |
| **A-LOAM 打印 `unsync message!` 然后退出** | 上游用**整数秒**做同步判断；如果 bag 里点云的整数秒跳变（比如录制时丢了 1 s），它会自杀 | 这是上游行为，不是 bug 引入的；重录包，或确认录制时 Gazebo 没卡顿 |
| `match_fraction` 很低 | 话题名不对 / 算法输出的是另一种消息 | 检查 `--algo` 与 `EST_TOPIC` 的对应关系（在 `bench_replay.launch.py` 顶部） |
| LOAM 的 ATE 大得离谱（>10 m） | 走廊/稀疏场景下真跑飞了 —— 这本身是实验结果 | 不要为了"好看"去调参数；如实报告，并在表里标 `Diverged=yes` |
| 回放跟不上（CPU 满） | Jetson 8 GB | `--rate 0.7`；或关掉 RViz（脚本默认已关） |
| 图没生成 | 没装 matplotlib | `sudo apt install python3-matplotlib`（数字不受影响） |

---

## 7. 复现性

- A-LOAM 钉在固定 commit（`demo.repos` / `install_deps.sh` 里的 `ALOAM_SHA`），
  报告里请写实际 commit：`git -C ros2_ws/src/A-LOAM-ROS2 rev-parse --short HEAD`
- LIO-SAM 用 `ros2` 分支：`git -C ros2_ws/src/LIO-SAM rev-parse --short HEAD`
- 每个 bag 的 `metadata.yaml` 里记录了录制时间与话题
- 每个 CSV 旁边的 `.meta.json` 记录了算法名、话题、帧数、时间戳范围
- 想重复多次看稳定性：`--repeats 3`（会录 3 份包，结果文件名带 `_r1/_r2/_r3`，汇总表里各自一行）
