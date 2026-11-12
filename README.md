# LOAM vs LIO-SAM：在同一份数据上比较纯几何与 IMU 融合

在 Gazebo 里搭一个走廊 / 房间，让同一台 16 线雷达小车按同一个动作跑，**录一次数据**，
再分别喂给 [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) 和
[A-LOAM](https://github.com/G-Apple1/A-LOAM-ROS2)（LOAM 的 ROS 2 移植），
用同一套指标比较两者。

一句话结论：

> **几何信息被拿走的场景（长走廊），IMU 把 ATE 从 2.11 m 压到 0.03 m（70 倍）；几何充足的场景两者都在毫米–厘米级，加 IMU 几乎没有收益。**

| 场景 | LOAM | LIO-SAM | 变化 |
|---|---|---|---|
| S0 名义运动 | 0.0095 m | **0.0047 m** | +51% |
| S1 高偏航 | 0.0051 m | **0.0035 m** | +31% |
| **S3 走廊退化** | **2.1095 m** | **0.0304 m** | **+98.6%** |
| S4 稀疏大厅 | 0.0105 m | **0.0044 m** | +59% |
| S5 长跑 180 s | **0.0042 m** | 0.0052 m | 打平 |
| **S2 高速** | **0.0112 m** | 0.8868 m | **−7799%（异常，见第 7 节）** |

> 指标为 ATE RMSE（定义见第 3 节），全部来自 [`results/tables/all_runs.csv`](results/tables/all_runs.csv)，
> 由脚本生成，报告里没有任何手写数字。

![S3 走廊](docs/img/s3_gazebo_view.png)

---

## 1. 运行环境

| 项 | 实测值 |
|---|---|
| 硬件 | NVIDIA **Jetson Orin Nano 8 GB**（aarch64） |
| 系统 / ROS | Ubuntu **24.04 LTS** + **ROS 2 Jazzy** |
| 仿真 | **Gazebo Harmonic 8.15**（经 `ros_gz`），RViz2 |
| LIO-SAM | 官方 `ros2` 分支，C++17，**GTSAM 4.2**（iSAM2）+ Ceres 2.2 |
| A-LOAM | ROS 2 移植，锁定 commit，**纯激光、无 IMU** |
| 实时因子 | **0.78**（同一块板子同时跑仿真与算法），所以各传感器只有标称频率的 ~78 % |

**仿真小车**（[`models/lio_bot/model.sdf`](ros2_ws/src/lio_sam_gz_bringup/models/lio_bot/model.sdf)）：

| 项 | 值 |
|---|---|
| 底盘 | 0.50 × 0.34 × 0.12 m，整备 6.1 kg |
| 轮 | r = 0.10 m，轮距 0.40 m |
| 激光雷达 | 16 线 × 360°，10 Hz，±15°，0.2 – 40 m，话题 `/points_raw` |
| IMU | 200 Hz，9 轴，话题 `/imu_raw` |
| 安装 | 雷达与 IMU **同位置同朝向**（外参为单位阵） |
| 真值 | 插件 `OdometryPublisher` → `/gt/odom`，约 80 Hz |

> 真值来自仿真器本身而不是带噪声的传感器，所以 ATE 衡量的是算法，不是参考源。

## 2. 比较是怎么实现的：录一次，跑两次

![ATE 汇总](docs/img/ate_summary.png)

```
              ┌── 2a 回放 → LIO-SAM（激光 + IMU + 因子图）
1 录一次 ─────┤
  Gazebo→bag  └── 2b 回放 → LOAM（只用激光，基线）
                                                    ↓
                                        3 定位：每帧 6 自由度位姿 → CSV
                                                    ↓
                                        4 分析：ATE / RPE / 鲁棒性 / 频率
```

**为什么这样设计**：两条算法消费的是**同一份 bag**，点云逐字节相同。因此表里的差异
不可能来自数据差异 —— 这是整个比较成立的前提。

`ros2 bag play --clock` 把包同时喂给两条管线，`traj_recorder` 把各自的位姿连同
**插值到同一时间戳的真值**写进 CSV（一场景一算法一个文件），最后才统一评测。

三条硬性约定：

- 真值按 lerp + slerp 插值到每帧雷达时间戳，**12 cm 杆臂显式补偿**
- 阈值**先定后测**（0.5 m 失效线、ATE 改善判据），不允许看到结果再调
- 体检脚本会拒绝**被截断、指标雷同、回放起点不一致**的运行 —— 这三类都会伪装成"结果"

## 3. 指标：ATE 怎么算

**ATE = Absolute Trajectory Error**。算法输出的世界坐标系是它自己定的，所以先做一次
刚体对齐，再逐帧量差。

**① 按时间戳配对**，只保留一对一匹配。

**② 求刚体变换**（最小二乘，闭式解）：

```
p̄  = mean(p_gt)          p̂̄ = mean(p_est)
H  = Σ (p̂ᵢ − p̂̄)(pᵢ − p̄)ᵀ           ← 去质心后的互协方差
H  = U Σ Vᵀ   ⇒   R = V·diag(1, 1, det(VUᵀ))·Uᵀ ,   t = p̄ − R·p̂̄
```

- `det` 那一项防止解出**镜像**（行列式 −1 是翻转，不是旋转）
- **不估尺度**：一条尺度错 10 % 的轨迹不能被"对齐"成完美结果

**③ 残差与指标**：

```
eᵢ  = ‖ pᵢ − (R·p̂ᵢ + t) ‖₂
ATE = √( (1/N) Σ eᵢ² )
```

用 RMSE 而不是平均，因为一段坏轨迹不该被大量好帧掩盖。单位米，越小越好。

同时报告：ATE 旋转分量、RPE（1 s 窗口）、终点误差、**误差 > 0.5 m 的帧数**、
输出频率，以及**未对齐的 raw RMSE**（用来看漂移绝对值）。

## 4. 六个场景

每个场景只改一个自变量：

| 场景 | 世界 | 轨迹长度 | 动作 | 隔离的变量 |
|---|---|---|---|---|
| **S0** nominal | 12 × 12 m 房间 | 36 m | 0.40 m/s 圆周 | 基准：几何充足 |
| **S1** fast yaw | 12 × 12 m 房间 | 27 m | ω = 0.90 rad/s | 单帧内的**运动畸变** |
| **S2** high speed | 12 × 12 m 房间 | 48 m | 1.2 m/s + 90° 原地转 | 帧间大位移 |
| **S3** corridor | 30 × 3 m 走廊 | 29 m | 沿轴往返 | **几何退化**：2 个自由度不可观测 |
| **S4** sparse hall | 20 × 20 m + 6 柱 | 36 m | r = 3 m 圆周 | 每帧特征稀疏 |
| **S5** long run | 12 × 12 m 房间 | 72 m | 9 圈 / 180 s | 累积漂移 |

场景定义在 [`experiments/scenarios/`](experiments/scenarios)，
录制前会先做**离线运动学 + 碰撞预检**，避免撞墙导致真值跳变（那批数据既不能算算法对、也不能算错）。

## 5. 结果

完整 12 行（6 场景 × 2 算法）见 [`results/tables/all_runs.csv`](results/tables/all_runs.csv)，
Markdown 表在 `results/tables/all_runs.md`，体检报告在 `results/tables/sanity.txt`。
复现这批数字的原始轨迹在 `results/raw/<场景>/<算法>.csv`（不入库，重跑即可再生）。

![S3 轨迹](docs/img/xy_s3_corridor.png)

## 6. 关键发现：几何退化处，IMU 才是必需的

走廊里四面墙中只有两面平行于行进方向，**沿轴位移与偏航在 scan-to-match 的雅可比里接近奇异**。
于是：

| S3 指标 | LOAM | LIO-SAM |
|---|---|---|
| ATE RMSE | 2.110 m | **0.030 m** |
| ATE 旋转 | 12.6° | **0.7°** |
| 终点误差 | 1.40 m | **0.027 m** |
| 误差 > 0.5 m 的帧 | **429 / 508** | **0** |

![S3 误差曲线](docs/img/err_s3_corridor.png)

橙线（LOAM）只在走廊中段一小截，蓝线（LIO-SAM）覆盖全程。**LOAM 估计的路径长度只有
真值的 18 %** —— 它不是"跑偏"，而是根本不往前走：沿 x 没有信息可用，求解器只能保留初值。
误差曲线也印证这一点：每掉一次头就重新错一次，在 0 与 3.6 m 之间来回摆。

这就是 IMU 的价值所在 —— 它约束的正是走廊拿掉的那两个方向。

## 7. 一个异常：S2 高速工况

| S2 指标 | LOAM | LIO-SAM |
|---|---|---|
| ATE RMSE | **0.011 m** | 0.887 m |
| RPE 平移 | 5.4 %/m | 121 %/m |
| 误差 > 0.5 m 的帧 | 0 | **282 / 282** |

**同一份包换一个回放倍速，结果从 0.887 m 变成 0.040 m** —— 所以它不稳定。
可能的原因是高动态下消息丢失或 IMU 预积分出现断档，**但我们没有定位到具体原因，也不作断言。**

如实报告它的意义在于：如果一个比较只挑对自己有利的场景收尾，那它是宣传，不是实验。
这条异常在报告里和执行摘要并列，没有藏在附录。

## 8. 复现（三条命令）

前置：Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic（Jetson 或普通 x86 均可）。

```bash
# 1) 装依赖 + 拉两个上游算法 + 打补丁 + 编译（一次性，约 5 分钟）
bash setup/install_deps.sh

# 2) 离线自检：不需要启动 ROS，约 50 秒
bash setup/check_all.sh          # 或 make check

# 3) 完整实验：6 场景 × 2 算法（约 35 分钟）
bash experiments/run_bench.sh    # 或 make bench
```

第 3 步会依次做：场景可行性预检 → 逐场景录制（Gazebo）→ 逐算法回放 → 评测 →
画图 → 数据体检 → 写 manifest，产出：

| 产物 | 内容 |
|---|---|
| `results/tables/all_runs.csv/.md` | 12 行主表 |
| `results/tables/sanity.txt` | 体检结论，`0 FAIL` 才可用于报告 |
| `results/figures/` | 轨迹图、误差曲线、ATE 柱状图、六宫格总览 |
| `results/raw/<场景>/<算法>.csv` | 原始轨迹（可复查） |
| `results/manifest.json` | 这批数字的出处（commit / 主机 / 包大小 / 行数） |

**只想快速验证某一段？**

```bash
# 只录包，不跑算法
bash experiments/run_bench.sh --record-only --record

# 复用已录好的包，只跑算法 + 评测
bash experiments/run_bench.sh --replay-only --rate 0.5

# 单场景、只跑一条算法
bash experiments/run_bench.sh --scenarios s3_corridor --replay-only --algo liosam
```

> `--rate` 是回放倍速。仿真实时因子只有 0.78，用 0.5–0.7 能让两条算法都吃满 CPU，
> 否则 LIO-SAM 可能因算力不足而丢帧 —— 这是第 7 节那条异常的来源之一。

**重新生成报告素材**（需要 `python3` + `matplotlib` + `python-pptx`）：

```bash
python3 report/make_part3_figures.py   # 六张场景俯视图 + ATE 图解
python3 report/make_ate_formula.py     # 公式排版图
python3 report/make_gazebo_view.py     # S3 的 Gazebo 风格视图
python3 report/build_deck.py           # 英文对比稿（20 页）
```

> `report/build_part3.py` 用于把结果填进课程模板，需要本地放一份
> `report/AAE4203_temp.pptx`；生成物不入库（见 `.gitignore`）。

## 9. 仓库结构

```
.
├── Makefile                    # make check / bench / deck / clean
├── demo.repos                  # 两个上游算法的版本锁定（vcstool）
├── setup/                      # 安装、补丁、自检（7 套，190+ 项检查）
├── experiments/                # 场景定义、跑测脚本、体检、评测
├── report/                     # 报告生成脚本（PPT 本身不入库）
├── ros2_ws/src/
│   ├── lio_sam_gz_bringup/     # 世界、小车模型、桥接、传感器格式转换
│   └── lio_sam_bench/          # 真值同步、轨迹记录、评测、画图
├── results/tables/             # 实测结果（入库，作为证据）
└── docs/                       # 文档索引、排错手册、图片
```

## 10. 文档与已知问题

| 文档 | 用途 |
|---|---|
| [`docs/README.md`](docs/README.md) | 一页索引：结构、命令、约定 |
| [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) | **排错优先查这份**：症状 → 根因 → 修复 |
| [`docs/LIO-SAM-intro-and-roadmap.md`](docs/LIO-SAM-intro-and-roadmap.md) | LIO-SAM 原理与实现路线 |
| [`experiments/README.md`](experiments/README.md) | 实验协议、注意事项 |

已知问题（细节都在排错手册里，这里只点名）：

- **静态 TF 必须用墙钟**（`use_sim_time:=False`），否则它要等 `/clock`，会晚于第一批点云
- **`baselinkFrame` 必须等于 `lidarFrame`**，否则 tf2 会绕道 `odom`，外参查不到 → GTSAM 秩亏崩溃
- **参数与 launch 从 `install/` 加载**，改了源码不 `colcon build` 等于没改（脚本会检查并提示）
- LIO-SAM 的建图节流 `mappingProcessInterval: 0.15`，在 8 Hz 点云下**每两帧只处理一帧**（设计如此，不是算力问题）

## 参考

- Shan, T., Englot, B., Meyers, D., Wang, W., Ratti, C., & Rus, D. (2020).
  *LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping.* IROS.
- Zhang, J., & Singh, S. (2014). *LOAM: Lidar Odometry and Mapping in Real-time.* RSS.
- Dellaert, F. *GTSAM* — https://github.com/borglab/gtsam
- A-LOAM (ROS 1) — https://github.com/HKUST-Aerial-Robotics/A-LOAM
- A-LOAM ROS 2 port — https://github.com/G-Apple1/A-LOAM-ROS2

---

MIT License · 课程项目（AAE4203 Guidance and Navigation）
