# LIO-SAM 介绍 · 学习路线 · 参考文档

> 面向大三学生（已学过线性代数、C++、ROS 基础）。
> 配套可跑的 demo 见仓库根目录 [`README.md`](../README.md)。

---

## 目录

1. [一句话定位](#1-一句话定位)
2. [它到底解决什么问题](#2-它到底解决什么问题)
3. [系统结构：4 个节点 / 2 个因子图](#3-系统结构4-个节点--2-个因子图)
4. [5 个关键技术点](#4-5-个关键技术点)
5. [输入要求（这是新手最容易翻车的地方）](#5-输入要求这是新手最容易翻车的地方)
6. [优点与局限](#6-优点与局限)
7. [和别的方案比一比](#7-和别的方案比一比)
8. [学习路线（4 周）](#8-学习路线4-周)
9. [前置知识清单](#9-前置知识清单)
10. [参考文档](#10-参考文档)
11. [9 个典型坑](#11-9-个典型坑)

---

## 1. 一句话定位

**LIO-SAM = LiDAR-Inertial Odometry via Smoothing and Mapping：把激光雷达和 IMU 紧耦合起来，用"因子图优化（smoothing）"而不是"滤波器"来做实时里程计 + 建图的开源框架。**

- 论文：Tixiao Shan, Brendan Englot, Drew Meyers, Wei Wang, Carlo Ratti, Daniela Rus,
  *LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping*, **IROS 2020**
  → [arXiv:2007.00258](https://arxiv.org/abs/2007.00258)
- 代码：[`TixiaoShan/LIO-SAM`](https://github.com/TixiaoShan/LIO-SAM)（`master` 分支 = ROS 1，
  **`ros2` 分支 = ROS 2**，本 demo 用的就是后者）
- 血统：LIO-SAM 的代码大量改编自作者的 **LeGO-LOAM**，特征提取部分来自 **LOAM**

三个词拆开看：

| 词 | 含义 | 对应实现 |
|---|---|---|
| **LiDAR** | 用机械旋转激光雷达做几何观测 | 16/32/64/128 线，要 `ring` 和 `time` 字段 |
| **Inertial** | 用 9 轴 IMU 做去畸变、初值、bias 估计、高频输出 | GTSAM IMU 预积分 |
| **Smoothing & Mapping** | 不用 EKF，而是把所有约束塞进因子图，用 iSAM2 增量优化 | `mapOptimization.cpp` |

> ⚠️ 注意区分：**"紧耦合"在 LIO-SAM 里指的是"把 IMU 预积分因子放进因子图"**，
> 而不是像 FAST-LIO 那样把点云残差直接对状态求导（那是另一种更紧的紧耦合）。

---

## 2. 它到底解决什么问题

给你一台"旋转激光雷达 + IMU"，让你回答两个问题：

1. **我现在在哪、朝哪**（6 DoF 位姿，100 Hz 级别的实时输出）→ 里程计
2. **周围长什么样**（三维点云地图）→ 建图

纯激光雷达（LOAM/LeGO-LOAM）的痛点：

- 雷达转一圈要 0.1 s，这 0.1 s 内机器人一直在动 → **点云是歪的（运动畸变）**，必须去畸变；
- 走廊 / 隧道 / 空旷场地几何退化 → 激光里程计漂移甚至发散；
- 只靠几何匹配，姿态的 roll/pitch 会慢慢漂。

IMU 恰好补上：短时间内角度变化极准（高频）、能感知重力方向（roll/pitch 绝对可观）。
把两者放进**同一个优化问题**里互相约束，就是 LIO。

而 LIO-SAM 的具体选择是：**不要滤波器，要因子图**。好处是回环、GPS、外部的任何相对/绝对观测都能
"作为一个因子插进去"，全局一致性地重优化；坏处是要维护一个增量式非线性优化器（iSAM2），有计算量。

---

## 3. 系统结构：4 个节点 / 2 个因子图

### 3.1 数据流

```
                ┌──────────────── 高频线程（IMU 频率 ~200 Hz）────────────────┐
                │                                                            │
   /imu/data ──►│  lio_sam_imuPreintegration                                 │
                │    ┌──────────────────────────────────────────┐            │
                │    │ ① IMU 预积分（GTSAM PreintegratedImu）     │            │
   /points  ───►│    │ ② 每来一帧激光里程计 → 加 IMU 因子+位姿因子 │            │
                │    │ ③ 输出 200 Hz 的 odom，并发布 odom→base_link│           │
                │    └──────────────────────────────────────────┘            │
                └────────────────────────────────────────────────────────────┘
                                        ▲
                                        │ lio_sam/mapping/odometry_incremental
                                        │
   /points ──► lio_sam_imageProjection   点云去畸变 + 投影成 range image
                    │  lio_sam/deskew/cloud_info  （含 ring 索引、range、IMU 初值 roll/pitch）
                    ▼
               lio_sam_featureExtraction  提取 edge(角点) / surf(面点)
                    │  lio_sam/feature/cloud_info
                    ▼
               lio_sam_mapOptimization   ★ 低频线程（~10 Hz，由 mappingProcessInterval 控制）
                    │  · 选关键帧（移动 >1 m 或转 >0.2 rad）
                    │  · 取周围关键帧拼成局部子图
                    │  · scan-to-map 迭代优化（LM，最多 30 次）
                    │  · iSAM2 增量优化：里程计因子 + GPS 因子 + 回环因子
                    │  · 发布 odom→lidar_link、全局地图、轨迹、当前帧
                    ▼
          /lio_sam/mapping/{odometry, path, map_global, cloud_registered, ...}
```

### 3.2 两个因子图（论文的核心）

| | **高频图**（`imuPreintegration.cpp`） | **全局图**（`mapOptmization.cpp`） |
|---|---|---|
| 频率 | IMU 频率（100–500 Hz） | 关键帧频率（~1 Hz 或更低） |
| 状态量 | IMU 位姿、速度、bias | 关键帧位姿 |
| 约束 | IMU 因子 + 激光里程计位姿因子 + bias 随机游走因子 | 里程计因子 + GPS 因子 + 回环因子 |
| 作用 | **给别人用的高频里程计**；估计 IMU bias | **全局一致性 + 地图** |
| 重置 | 每 100 个关键帧重建一次（防止越算越慢） | 全程维护 |

这个"**双图 + 定期重置**"是 LIO-SAM 能做到 10× 实时的核心设计。
面试/答辩常问的一句话是：**"为什么需要高频图？"** —— 因为全局图要几十毫秒，机器人控制需要 IMU 频率的位姿。

---

## 4. 5 个关键技术点

### 4.1 用 IMU 预积分给点云去畸变（deskew）

对第 `i` 个点，用它的相对时间 `time∈[0, 0.1]` 在 IMU 积分出的旋转序列里插值，得到
`T_start⁻¹ · T_i`，把点从"打点时刻的雷达系"搬回"这一帧起始时刻的雷达系"。

代码：`imageProjection.cpp` 的 `deskewPoint() / findRotation()`。
**这就要求点云里必须有 `time` 字段**（见第 5 节）。

### 4.2 range image 投影

把 3D 点投成一张 `N_SCAN × Horizon_SCAN` 的"距离图像"（行=线号 `ring`，列=水平角），
每个格子只保留最近的点 → 天然去遮挡、天然有邻域结构，比 KD-Tree 更适合后续特征提取。

代码：`imageProjection.cpp` 的 `projectPointCloud()`。

### 4.3 特征提取：edge + surf

在 range image 里判断局部平滑度 `c = ‖Σp_i‖ / (n·‖p‖)`：
`c` 大的是**角点(edge)**，小的是**面点(surf)**。然后做"每行/每段最多取几个"的均匀化，
避免特征全挤在一面墙上。

代码：`featureExtraction.cpp`。

### 4.4 scan-to-map（局部子图）而不是 scan-to-global-map

这是实时性的关键：**永远只和"当前位姿周围的关键帧拼出来的子图"匹配**，
而不是和整张全局地图匹配。全局地图只用于优化后的重投影。

代码：`mapOptmization.cpp` 的 `extractSurroundingKeyFrames() / scan2MapOptimization()`。
参数 `surroundingKeyframeSearchRadius`、`surroundingKeyframeDensity`。

### 4.5 回环 + GPS 因子

- 回环：沿用 LeGO-LOAM 的做法，按位置/时间/数量筛历史关键帧，拼子图 + ICP，
  过了 `historyKeyframeFitnessScore` 就往图里加一个 `BetweenFactor`。
  **ICP 很慢**，所以官方建议 `rosbag play -r 1`。
- GPS：`navsat` 模块把 NavSatFix 转成 ENU 下的 `nav_msgs/Odometry`（`odometry/gps`），
  按协方差门限筛一下后加 `GPSFactor`。

这两个都是**可选**的，demo 里默认都关掉，跑通后再打开（见根目录 README 的练习）。

---

## 5. 输入要求（这是新手最容易翻车的地方）

> LIO-SAM 原 README 里明确写了 "must read"，我把它翻译成人话 + 加了排错提示。

### 5.1 点云必须带 `ring` 和 `time`

| 字段 | 含义 | 没有会怎样 |
|---|---|---|
| `x, y, z` | 点坐标（米，雷达系，REP-103） | 必须有 |
| `intensity` | 反射强度 | 可缺，但建议有 |
| `ring` | 这是第几线（0..N_SCAN-1） | 报 `Point cloud ring channel not available`；ROS 2 版对 `sensor: velodyne` 有"用垂直角反算"的兜底 |
| `time` | 该点相对本帧起始时刻的偏移（秒，0~0.1） | 报 `deskew function disabled, system will drift significantly`，高速运动会歪 |

**这就是本 demo 需要 `sensor_prep.py` 的原因**：Gazebo 的 `gpu_lidar` 只能输出
`x,y,z,intensity`，`ring`/`time` 得自己补。

### 5.2 IMU 必须是 9 轴，且要有合法 `orientation`

LIO-SAM 用 IMU 的四元数做**重力对齐初始化和回环/GPS 的航向**。
带 6 轴 IMU（像 Ouster 内置那个）的会直接退出：

```
Invalid quaternion, please use a 9-axis IMU!
```

### 5.3 两个旋转外参，别填错

```yaml
extrinsicRot:  # 把 IMU 的角速度/加速度转到雷达系
extrinsicRPY:  # 把 IMU 的姿态转到雷达系（很多 IMU 的 acc 系和姿态系不一样）
extrinsicTrans: [x, y, z]  # IMU 原点在雷达系下的位置
```

最常见的翻车现象：**车一启动就上下乱跳 / 地图炸开** → 八成是 `extrinsicRot` 错了，
加速度计测到的重力符号反了。官方 README 的 "Jumping up and down" 条目说的就是它。

**省事做法**：把 IMU 和雷达装在同一个点、同一个朝向（本 demo 的 `model.sdf` 就是这么做的），
三个外参全填单位矩阵 / 零向量。

### 5.4 时间同步

激光和 IMU 的时间戳必须在同一个时钟里、且不能乱序。
仿真里靠 `/clock` + `use_sim_time:=true` 保证；实测里靠 PTP/PPS 或至少同一个机器时钟。
现象：**轨迹呈锯齿状（zigzag）** → 时间戳没同步。

---

## 6. 优点与局限

**优点**

- 精度好：在 KITTI / 自采数据上明显优于纯 LOAM 系；
- **架构干净**：4 个节点、单一职责，"想加传感器就加因子"，非常适合当**教学/科研基线**；
- 支持 GPS 因子、回环、地图保存（`/lio_sam/save_map`）；
- 实时（论文报 10× 实时）；
- ROS 2 分支官方在维护（虽然功能比 ROS 1 少一点：navsat launch、rviz 配置不全）。

**局限（写报告/答辩时说这些显得懂）**

- **只支持机械旋转式雷达**，非重复扫描的固态雷达 / 纯 Livox 需要改代码；
- 强依赖 **9 轴 IMU** 的质量与内外参标定；
- 回环用的是 **ICP**，慢且对初值敏感，工程上常换成 Scan Context；
- 前端还是"特征法"，**弱纹理/纯平面/长走廊**容易退化（代码里 `isDegenerate` 只是打了个标记）；
- 没有动态物体剔除，人走来走去会把点云拖影进地图；
- ROS 2 分支的 `run.launch.py` 里 rviz 配置比较简陋，得自己补 Display。

---

## 7. 和别的方案比一比

| 方案 | 融合方式 | 雷达要求 | 输出频率 | 适合谁 |
|---|---|---|---|---|
| **LIO-SAM** | 因子图（松-紧折中，IMU 预积分入图） | 机械旋转，要 `ring`+`time` | 全局图 ~1–10 Hz + IMU 图 200 Hz | **想学"图优化 + LIO"的人**（本 demo） |
| LeGO-LOAM | 无 IMU（纯激光 + 地面分割） | 机械旋转 | 10 Hz | 入门激光 SLAM |
| FAST-LIO2 | **ESKF 紧耦合**（点面残差直接进滤波） | 任意（含固态、Livox） | 50–100 Hz | 要轻、要快、要"什么雷达都能跑" |
| Point-LIO | 逐点更新 + 无重建，高动态 | 任意 | 很高 | 高机动 / 无人机 |
| GLIM | 全局优化 + GPU 体素匹配 | 任意 | — | 追求建图质量 |
| KISS-ICP | 纯几何 ICP，无 IMU | 任意 | 10 Hz | 5 分钟上手当基线 |

> 学习顺序建议：**先 KISS-ICP 建立"里程计就是配准"的直觉 → 再 FAST-LIO2 理解滤波版紧耦合 →
> 再 LIO-SAM 理解图优化版。** 三条路线都跑过，LIO 这一块基本就有全景了。

---

## 8. 学习路线（4 周）

> 假设每周投入 6–10 小时。**核心原则：先跑起来 → 再读代码 → 最后读公式。**
> 顺序反过来的话，第 2 天就会放弃。

### 第 0 阶段（半天）：环境

- [ ] Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic（本 demo 的 `setup/install_deps.sh` 会装好）
- [ ] 会 `ros2 topic list/echo/hz`、`ros2 run rqt_graph rqt_graph`、`ros2 bag record/play`
- [ ] 看得懂 TF 树：`ros2 run tf2_tools view_frames`

### 第 1 周：先建立直觉（不要碰公式）

- [ ] 跑通 Demo 1（`gazebo_lab1.launch.py`），在 RViz 里看 `/points`
- [ ] 跑通 Demo 2（`lio_sam_gazebo.launch.py`），看 LIO-SAM 建出来的地图和轨迹
- [ ] `ros2 topic hz` 量一遍：点云 10 Hz、IMU 200 Hz、`map_global` 0.2 Hz
- [ ] `rqt_graph` 截图，**自己画出 4 个节点的数据流**（照着第 3 节对答案）
- [ ] 故意把 `params_gazebo.yaml` 里 `N_SCAN` 改成 32，观察崩掉 → **理解"参数必须和传感器匹配"**
- [ ] 把 `imuTopic` 改成一个不存在的话题，观察它卡在 `Waiting for IMU data ...`

**这一周的目标**：知道"每个话题是谁发的、谁收的、频率多少"。

### 第 2 周：读代码（按数据流顺序，只读入口函数）

按这个顺序读，每个函数只追主逻辑，**跳过所有可视化和调试代码**：

1. `imageProjection.cpp`
   - `imuHandler()` → `imuConverter()`（外参在这里生效）
   - `cloudHandler()` → `cachePointCloud()`（字段检查！） → `deskewInfo()` → `projectPointCloud()` → `cloudExtraction()`
   - 重点看懂：`rangeMat` 是怎么被填的、`columnIdn` 怎么由水平角算出来
2. `featureExtraction.cpp`
   - `calculateSmoothness()`、`markOccludedPoints()`、`extractFeatures()`：一行扫描线上怎么挑点
3. `imuPreintegration.cpp`
   - 两个类：`IMUPreintegration`（高频图）和 `TransformFusion`（把激光里程计和 IMU 增量拼起来，发 TF）
   - 重点：`odometryHandler()` 里"先积分、再加因子、再优化、再重播"的循环
4. `mapOptmization.cpp`（最长，挑 4 个函数）
   - `laserCloudInfoHandler()`（主循环入口）
   - `extractSurroundingKeyFrames()` + `scan2MapOptimization()`
   - `saveKeyFramesAndFactor()`（什么时候加因子、加什么因子）
   - `publishOdometry()`（TF 是谁发的，记住：`odom→lidar_link`）

> 读代码技巧：在 RViz 里加一路 `/lio_sam/deskew/cloud_deskewed`，一边读一边看，
> 比纯看代码快 3 倍。

### 第 3 周：读公式（只读 4 个）

| 主题 | 读什么 | 到什么程度 |
|---|---|---|
| IMU 预积分 | GTSAM 预积分推导 / [Forster 2017 ICRA](https://arxiv.org/abs/1512.02363) | 能说清"为什么预积分能避免重复积分、bias 更新后怎么修正" |
| 因子图 & iSAM2 | [GTSAM 官方 Primer](https://gtsam.org/tutorials/intro.html) + 《Factor Graphs for Robot Perception》第 1–3 章 | 会用 `PriorFactor / BetweenFactor / ImuFactor / GPSFactor` 手写一个 2D 位姿图 |
| 点到线/点到面残差 | LOAM 论文第 IV 节 + LIO-SAM 的 `LMOptimization()` | 能写出 edge/surf 的残差和雅可比形状（6×1） |
| 退化检测 | `LMOptimization()` 里 `cv::eigen(matAtA, ...)` 那段 | 知道它为什么要对 `matAtA` 做特征值分解 |

**强烈建议做的小实验**：用 GTSAM Python 写一个 5 个位姿的 2D 位姿图（`python3-gtsam` 装好就有），
把 `BetweenFactor` 加进去优化一次，你会**瞬间**理解 LIO-SAM 的 `saveKeyFramesAndFactor()`。

### 第 4 周：动手改（这一步才是真学会）

选 2–3 个做，写进报告：

- [ ] **录 bag 离线跑**：`ros2 bag record /points /imu/data /tf /tf_static`，然后 `ros2 bag play`。体会"离线可复现"。
- [ ] **打开回环**：`loopClosureEnableFlag: true`，绕圈 2 圈以上，观察 `map_global` 是否被"拉回来"。
- [ ] **加 GPS 因子**：Gazebo 里有 `navsat` 传感器，桥接成 `NavSatFix`，用 `navsat_transform_node`（`robot_localization`）转成 `odometry/gps`，把 `gpsTopic` 指向它。
- [ ] **对比轨迹**：`/lio_sam/mapping/odometry` vs `/odom_gz`（Gazebo 真值），用 Python 画 XY 曲线 + 算 ATE/RPE（`evo` 或手写 20 行）。
- [ ] **故意搞坏再修**：把 `extrinsicRot` 的 z 轴取反，观察"跳跃"现象；再改回来。**这一条比读 10 篇博客有用。**
- [ ] **换数据集**：下载 KITTI 的 00 序列（有 IMU 和 Velodyne HDL-64E），按官方 README 改 `N_SCAN: 64`、`downsampleRate: 2`、`extrinsicTrans`。

### 学完之后你应该能回答的问题（自测）

1. LIO-SAM 为什么要有两张因子图？各自的状态量和约束是什么？
2. 点云去畸变时，`time` 字段的单位和范围是什么？没有它会怎样？
3. `extrinsicRot` 和 `extrinsicRPY` 为什么要分开两个？
4. 为什么 LIO-SAM 只和局部子图匹配，不和全局地图匹配？
5. 关键帧是怎么选的？`surroundingkeyframeAddingDistThreshold` 调大调小分别有什么后果？
6. 回环检测用了什么？为什么官方建议 `-r 1` 播放？
7. LIO-SAM 在长走廊里会发生什么？代码里 `isDegenerate` 是怎么被用到的？

---

## 9. 前置知识清单

按"必须 / 加分"分级，别贪多：

| 主题 | 必须到什么程度 | 推荐材料 |
|---|---|---|
| 线性代数 | 旋转矩阵、正交矩阵、特征值分解、最小二乘 `Ax=b` | 任何一本工科线代 |
| 3D 旋转表示 | 四元数 ↔ 旋转矩阵 ↔ 欧拉角互换；左乘右乘的区别 | [《视觉SLAM十四讲》第3讲] / [Rotation Converter](https://www.andre-gaschler.com/rotationconverter/) |
| SE(3) / 李群李代数 | 只要能理解"位姿的加法要用指数映射"，**不用会推** | 十四讲第4讲 |
| ROS 2 | node/topic/service/parameter、**TF2 与 `use_sim_time`**、launch、colcon | [ROS 2 Jazzy 官方 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) |
| C++ | 类、模板、`std::shared_ptr`、Eigen、多线程 `std::thread/mutex` | — |
| CMake / colcon | 会看 `CMakeLists.txt`、会加依赖 | ROS 2 `ament_cmake` 教程 |
| PCL | `PointCloud<T>`、`VoxelGrid`、`KdTreeFLANN`、`fromROSMsg` | [PCL 官方教程](https://pointclouds.org/documentation/tutorials/) |
| 非线性优化 / 因子图 | **必须**：知道"最小化 Σ‖残差‖²_Σ"、高斯牛顿、稀疏性 | GTSAM Primer（80 页，一周能啃完） |
| IMU 误差模型 | 加分：白噪声、随机游走 bias、预积分 | Forster 2017 |

---

## 10. 参考文档

### 10.1 原始论文与作者资料

| 资料 | 链接 | 备注 |
|---|---|---|
| LIO-SAM 论文（IROS 2020） | https://arxiv.org/abs/2007.00258 | 必读，8 页，配图很清楚 |
| 作者主页 / 项目页 | https://github.com/TixiaoShan/LIO-SAM | README 的 "must read" 两节必看 |
| 作者的视频讲解 | https://www.youtube.com/watch?v=A0H8CoORZJU | README 里给的 demo 视频 |
| LeGO-LOAM（前作） | https://github.com/RobustFieldAutonomyLab/LeGO-LOAM | 回环、地面分割都来自这 |
| SC-LIO-SAM（+Scan Context 回环） | https://github.com/gisbi-kim/SC-LIO-SAM | 想改回环就从这抄 |

### 10.2 代码

| 资料 | 链接 |
|---|---|
| LIO-SAM 仓库（`master` = ROS 1，**`ros2` = ROS 2**） | https://github.com/TixiaoShan/LIO-SAM |
| 本 demo 用的就是 `ros2` 分支 | `git checkout ros2` |
| Autoware 官方 LIO-SAM 集成教程 | https://docs.autoware.org/1.8.0/tutorials/integrating-autoware/creating-maps/open-source-slam/lio-sam/ |

### 10.3 ROS 2 / Gazebo 官方文档

| 资料 | 链接 |
|---|---|
| ROS 2 Jazzy 安装（Ubuntu 24.04） | https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html |
| ROS 2 Jazzy Tutorials | https://docs.ros.org/en/jazzy/Tutorials.html |
| TF2 概念与教程 | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html |
| REP-103（坐标轴约定）/ REP-105（frame 约定） | https://www.ros.org/reps/rep-0103.html · https://www.ros.org/reps/rep-0105.html |
| Gazebo Harmonic 文档 | https://gazebosim.org/docs/harmonic/ |
| Gazebo Harmonic + ROS 2 安装配对 | https://gazebosim.org/docs/harmonic/ros_installation |
| `ros_gz` 仓库（bridge 的 YAML 配置就在这里） | https://github.com/gazebosim/ros_gz |
| `ros_gz_bridge` README | https://docs.ros.org/en/jazzy/p/ros_gz_bridge/ |
| `ros_gz_sim_demos`（官方小 demo，排查环境很好用） | https://docs.ros.org/en/jazzy/p/ros_gz_sim_demos/ |

### 10.4 优化 / 数学

| 资料 | 链接 | 备注 |
|---|---|---|
| GTSAM 官网 + 安装 | https://gtsam.org/get_started/ | Ubuntu 24.04 直接 `apt install libgtsam-dev` |
| **GTSAM Primer（强烈推荐）** | https://gtsam.org/tutorials/intro.html | 80 页，读完因子图就通了 |
| 《Factor Graphs for Robot Perception》 | https://doi.org/10.1561/2300000043 | Dellaert & Kaess，系统教材 |
| iSAM2 论文 | https://doi.org/10.1177/0278364911430419 | 想懂增量优化再看 |
| IMU 预积分（Forster 2017） | https://arxiv.org/abs/1512.02363 | 理解 `ImuFactor` 的前提 |
| LOAM 论文（RSS 2014） | Zhang & Singh, *LOAM: Lidar Odometry and Mapping in Real-time* | 特征提取的祖宗 |
| A-LOAM（LOAM 的干净重实现） | https://github.com/HKUST-Aerial-Robotics/A-LOAM | 比 LOAM 原码好读 10 倍 |

### 10.5 数据集（想脱离仿真做实验）

| 数据集 | 链接 | 为什么推荐 |
|---|---|---|
| KITTI Odometry | https://www.cvlibs.net/datasets/kitti/eval_odometry.php | 有 Velodyne HDL-64E + IMU，LIO-SAM README 给了参数 |
| Newer College | https://ori-drs.github.io/newer-college-dataset/ | 手持雷达，室内外都有 |
| NCLT | http://robots.engin.umich.edu/nclt/ | 长时间、有 GPS，适合试 GPS 因子 |
| MulRan | https://sites.google.com/view/mulran-pr/home | 城市大场景 + 回环 |
| Boreas | https://boreas.utias.utoronto.ca/ | 四季/雪天，考察鲁棒性 |

### 10.6 对照方案（都值得跑一遍）

- **KISS-ICP**：https://github.com/PRBonn/kiss-icp —— 20 行核心代码，最快的"能跑"体验
- **FAST-LIO2**：https://github.com/hku-mars/FAST_LIO · 论文 https://arxiv.org/abs/2107.06829
- **Point-LIO**：https://github.com/hku-mars/Point-LIO
- **GLIM**：https://github.com/koide3/glim
- **A-LOAM**：https://github.com/HKUST-Aerial-Robotics/A-LOAM

### 10.7 中文资料（按关键词自己搜，链接易失效故不硬编码）

- 关键词：`LIO-SAM 代码解读` / `LIO-SAM 论文笔记` / `LIO-SAM 因子图` / `GTSAM 入门`
- 书籍：《视觉SLAM十四讲》（高翔）第 3、4、9 章 —— 坐标变换和优化部分够用了
- 深蓝学院 / 古月居的 LiDAR SLAM 课程（付费，选听）
- **提醒**：网上很多解读是 ROS 1 版的行号，和 `ros2` 分支对不上，看代码以仓库为准。

---

## 11. 9 个典型坑

| 现象 | 八成原因 | 怎么办 |
|---|---|---|
| `Point cloud ring channel not available` | 点云缺 `ring` | 本 demo 的 `sensor_prep.py` 会补；自己接雷达就用驱动原生输出 |
| `deskew function disabled` | 点云缺 `time` | 同上；或改 `imageProjection.cpp` 用 `t` 字段名 |
| `Invalid quaternion, please use a 9-axis IMU!` | IMU 没姿态（6 轴/仿真没发） | 换 9 轴 IMU；或像 `sensor_prep.py` 那样用重力补 roll/pitch |
| 一启动就上下乱跳 / 地图炸开 | `extrinsicRot` 错（重力符号反了） | 把 `extrinsicRot` 第 3 行/列取反试试；或干脆让 IMU 与雷达同向 |
| 轨迹呈锯齿状 | 激光和 IMU 时间戳没同步 | 用 `/clock` + `use_sim_time`；实测用 PTP |
| `Waiting for IMU data ...` 一直卡 | IMU 频率太低 / 时间戳在点云之后 | IMU 至少 200 Hz，且要覆盖整帧时间窗 |
| `Not enough features!` | 场景太空旷 / `lidarMaxRange` 太大 / `N_SCAN` 不匹配 | 加墙加箱子；把 `lidarMinRange/MaxRange` 调到合理值 |
| 编译报 `find_package(GTSAM)` 失败 | 没装 GTSAM 或版本 <4 | `sudo apt install libgtsam-dev` |
| 编译报 `gtsam_unstable/... No such file` | Ubuntu 的 gtsam 包不含 unstable 模块 | 用本 demo 的 `setup/patch_lio_sam.sh` |

---

## 附：本 demo 与官方代码的关系

| 官方 LIO-SAM | 本 demo 的做法 |
|---|---|
| 需要真实雷达 / rosbag | 用 Gazebo 仿真生成 `/points` 与 `/imu/data` |
| 期望点云自带 `ring`/`time` | `scripts/sensor_prep.py` 补这两个字段（这是唯一"额外"的代码） |
| `run.launch.py` 自带 `robot.urdf.xacro` + rviz | 用 `lio_sam_gazebo.launch.py` 提供等价的静态 TF + 专门调过的参数 |
| `params.yaml` 默认 Ouster 64 线 | `config/params_gazebo.yaml` 改成 Velodyne 16 线 |

**除了 `sensor_prep.py` 和参数文件，LIO-SAM 一行代码都不用改**（除了为编译通过打的 3 行补丁）。
