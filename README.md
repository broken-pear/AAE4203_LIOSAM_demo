# AAE4203_LIOSAM_demo - LIO-SAM on ROS 2 Jazzy with Gazebo + RViz (teaching demo)

> **目标**：一条命令让一台"16 线激光雷达 + 9 轴 IMU"的小车在 Gazebo 里绕圈，
> 点云实时喂给 **LIO-SAM**，在 **RViz2** 里看到建出来的三维地图和轨迹。
> 除了一个 90 行的格式转换脚本，**LIO-SAM 一行代码都不用改**。

📖 理论、学习路线、参考文献都在 **[`docs/LIO-SAM-intro-and-roadmap.md`](docs/LIO-SAM-intro-and-roadmap.md)**

---

## 目录

- [0. 这套东西能跑出什么](#0-这套东西能跑出什么)
- [1. 目录结构](#1-目录结构)
- [2. 环境要求（先看这一节，能省一天）](#2-环境要求先看这一节能省一天)
- [2.5 把代码推到 GitHub，再在目标机器 clone（推荐）](#25-把代码推到-github再在目标机器-clone推荐)
- [3. 安装：三步](#3-安装三步)
- [4. Demo 1：只看 Gazebo + RViz 的传感器](#4-demo-1只看-gazebo--rviz-的传感器)
- [5. Demo 2：跑 LIO-SAM 建图](#5-demo-2跑-lio-sam-建图)
- [5.5 Demo 3：跑 LOAM baseline](#55-demo-3跑-loam-baseline)
- [5.6 Demo 4 + 对比实验：谁更精确，差多少](#56-demo-4--对比实验谁更精确差多少)
- [6. 数据流（谁发谁收）](#6-数据流谁发谁收)
- [7. 手动逐个终端启动（教学用）](#7-手动逐个终端启动教学用)
- [8. 为了跑通我做了什么](#8-为了跑通我做了什么)
- [9. 排错 FAQ](#9-排错-faq)
- [10. 练习](#10-练习)
- [11. 命令速查](#11-命令速查)

---

## 0. 这套东西能跑出什么

```
gazebo_lab1.launch.py     → Gazebo 里有台小车在方形大厅里绕圈，RViz 里看到 360° 点云
lio_sam_gazebo.launch.py  → 上面那些 + LIO-SAM 输出的地图/轨迹/TF
```

RViz 里应该看到：

| Display | 话题 | 长什么样 |
|---|---|---|
| Map (global) | `/lio_sam/mapping/map_global` | 淡蓝色点云，把整个房间"描"出来，0.2 Hz 刷新 |
| Current scan | `/lio_sam/mapping/cloud_registered` | 橙色的当前帧，跟着车走 |
| Trajectory | `/lio_sam/mapping/path` | 绿色轨迹线，应该是一个规整的圆 |
| TF | — | `map → odom → base_link → lidar_link` |

如果轨迹是圆、地图和 Gazebo 里的房间对得上 —— **你就跑通了**。

---

## 1. 目录结构

```
AAE4203_LIOSAM_demo/
├── README.md                          ← 你正在看的
├── docs/
│   ├── LIO-SAM-intro-and-roadmap.md        ← 介绍 / 原理 / 4 周学习路线 / 参考文献
│   └── TROUBLESHOOTING.md                  ← 18 个实战坑：症状→根因→修复→验证
├── experiments/                       ← ★ 对比实验：场景脚本 + 一键跑（详见其 README）
│   ├── README.md                           实验协议 / 指标定义 / 排错
│   ├── run_bench.sh                        录包 → 回放 → 评测 → 出表
│   └── scenarios/s0..s5.yaml               6 个场景的动作脚本
├── results/                           ← 实验产物：raw CSV / json / tables / figures
├── setup/
│   ├── check_env.sh                   ← 体检：缺什么一目了然
│   ├── install_deps.sh                ← 一件安装：apt + clone 两套算法 + 打补丁 + 编译
│   ├── patch_lio_sam.sh               ← LIO-SAM 在 Ubuntu 24.04 上编过的 3+2 行补丁
│   ├── patch_aloam.sh                 ← A-LOAM 的 Ceres 2.2 补丁（LocalParameterization→Manifold）
│   ├── _selfcheck.py                  ← 无 ROS 也能跑的静态校验（128 项）
│   ├── _test_metrics.py               ← 评测器的解析验证（ATE/RPE/关联）
│   ├── _test_pipeline.py              ← CSV → 表 → 图 → PPT 全链路自检
│   ├── _test_nodes.py                 ← gt_sync/traj_recorder 的 stub 自检
│   └── _test_pack.py                  ← ring/time + PointCloud2 布局单测
└── ros2_ws/
    └── src/
        ├── LIO-SAM/                   ← 跑 install_deps.sh 后自动 clone（官方 ros2 分支）
        ├── A-LOAM-ROS2/               ← 同上，A-LOAM 的 ROS 2 移植（pin 到固定 commit）
        ├── lio_sam_gz_bringup/        ← ★ 胶水包：仿真场景 + 传感器适配 + 两条算法的 launch
        │   ├── launch/
        │   │   ├── gazebo_lab1.launch.py       Demo 1：Gazebo + 传感器 + RViz
        │   │   ├── lio_sam_gazebo.launch.py    Demo 2：Demo 1 + LIO-SAM
        │   │   ├── lio_sam_nodes.launch.py     只起 LIO-SAM 4 节点（回放/对比复用）
        │   │   ├── loam_gazebo.launch.py       Demo 3：Demo 1 + A-LOAM baseline
        │   │   └── loam_nodes.launch.py        只起 A-LOAM 3 节点
        │   ├── config/gz_bridge.yaml           Gazebo ⇄ ROS 桥接（含真值 /gt/odom）
        │   ├── config/params_gazebo.yaml       LIO-SAM 参数（按 16 线雷达调好）
        │   ├── models/lio_bot/model.sdf        差速小车 + 16 线雷达 + IMU + 真值插件
        │   ├── worlds/                         lab1_room / corridor / hall_sparse
        │   ├── rviz/                           lab1_sensors / lio_sam_slam / loam_slam
        │   └── scripts/
        │       ├── sensor_prep.py              ★ 点云加 ring/time + IMU 兜底
        │       ├── patrol_driver.py            匀速绕圈（Demo 用）
        │       └── scripted_driver.py          按脚本分段开车（实验用）
        └── lio_sam_bench/             ← ★ 测量包：真值同步 + 录轨迹 + 评测 + 画图
            ├── launch/compare_gazebo / bench_replay / record_bag
            ├── scripts/gt_sync.py / traj_recorder.py / evaluate.py / make_figures.py
            └── config/bench.yaml
```

---

## 2. 环境要求（先看这一节，能省一天）

| 项目 | 要求 | 说明 |
|---|---|---|
| 操作系统 | **Ubuntu 24.04 (noble)** | ROS 2 Jazzy 的官方支持平台 |
| ROS 2 | **Jazzy** | 24.04 对应的就是它 |
| Gazebo | **Harmonic**（`gz sim` 8） | 通过 `ros-jazzy-ros-gz` 装，**不要**单独装 Gazebo Classic（那是 ROS 1 时代的东西） |
| GTSAM | **4.2**，`apt install libgtsam-dev` | ★ 好消息：Ubuntu 24.04 universe 自带，**不用编 GTSAM、不用加 PPA** |
| 显卡 | 有 GPU 最好；没有也能跑（见下） | Gazebo 的 `gpu_lidar` 需要渲染 |

### ❗ 不能在 Windows 原生跑

ROS 2 / Gazebo / RViz 生态基本都在 Linux 上。Windows 上请用 **WSL2 + Ubuntu 24.04**：

```powershell
# 管理员 PowerShell
wsl --install -d Ubuntu-24.04
```

装完重启，进 Ubuntu，然后按下面的步骤走。WSL2 自带 **WSLg**，`rviz2` 和 Gazebo 的窗口能直接弹出来。

> **WSL2 跑 Gazebo 的两个小坑**
> - 如果 Gazebo 黑屏 / 崩在渲染上 → `export LIBGL_ALWAYS_SOFTWARE=1`（软件渲染，慢但能跑）。
> - 如果 RViz 弹不出来 → 确认 Windows 有 WSLg（Win10 21H2+ / Win11），或者装个 X Server 设 `DISPLAY`。

### 已经在别的机器上装好 ROS 2 了？

那可以直接跳到第 3 步。只要确认这三件事：

```bash
ls /opt/ros/jazzy/setup.bash      # ROS 2 Jazzy 在
apt policy ros-jazzy-ros-gz       # Gazebo 桥接包能装
apt policy libgtsam-dev           # GTSAM 4.2 能装
```

---

## 2.5 把代码推到 GitHub，再在目标机器 clone（推荐）

在 Windows/Mac 上编辑、git 推送，再在 Ubuntu（含 Jetson）上 clone —— 比拷压缩包干净，
而且 `.gitattributes` 已经把 `*.sh / *.py` 强制成 **LF**，不会出现
`bash: ./install_deps.sh: /bin/bash^M: bad interpreter` 这种 Windows 换行坑。

### 2.5.1 本机（Windows / Mac）推送

先在 GitHub 上建一个**空仓库**（不要勾 README / .gitignore / license），然后：

```powershell
# Windows PowerShell
cd "D:\...\AAE4203_LIOSAM_demo"          # 就是本文件所在目录

git init
git add .
git commit -m "LIO-SAM x ROS2 Jazzy x Gazebo demo"
git branch -M main
git remote add origin https://github.com/<你的用户名>/<仓库名>.git
git push -u origin main
```

只想先备份到本地的话，`git commit` 完就够，`remote/push` 可以以后再加。

> ⚠️ **不要**把 `ros2_ws/src/LIO-SAM/` 提交进去：它是 `install_deps.sh` 在目标机器上
> clone 的第三方代码，`.gitignore` 里已经排除了。这样本仓库保持 MIT、体积只有几十 KB。

### 2.5.2 目标机器（Ubuntu / Jetson）拉下来

```bash
cd ~
git clone https://github.com/<你的用户名>/<仓库名>.git AAE4203_LIOSAM_demo
cd AAE4203_LIOSAM_demo

# 之后照第 3 步走就行（install_deps.sh 会自动把 LIO-SAM clone 到 ros2_ws/src/）
bash setup/check_env.sh
bash setup/install_deps.sh
```

**`git clone` 时务必把目录名指定成 `AAE4203_LIOSAM_demo`**（命令末尾那个参数），
这样和 README / demo 里的路径保持一致；否则你仓库叫别的名字，后面还要自己改路径。

### 2.5.3 想用 `vcs` 管理依赖（可选）

仓库里带了一个 `demo.repos`，也可以这样拉源码：

```bash
sudo apt install python3-vcstool
cd ~/AAE4203_LIOSAM_demo
vcs import ros2_ws/src < demo.repos        # 等价于 install_deps.sh 里的 git clone
bash setup/install_deps.sh                 # 后面照样跑（已存在的目录会跳过 clone）
```

---

## 3. 安装：三步

```bash
# 准备：把整个 AAE4203_LIOSAM_demo 目录放到 Linux 下（比如 ~/AAE4203_LIOSAM_demo）
export ROS_DEMO=~/AAE4203_LIOSAM_demo
cd $ROS_DEMO
```

### 第 1 步：配好 ROS 2 环境

```bash
source /opt/ros/jazzy/setup.bash
```

> 如果 `/opt/ros/jazzy` 不存在，先装 ROS 2（约 15 分钟，几个 GB）：
> ```bash
> sudo apt update && sudo apt install -y ros-jazzy-desktop
> ```
> 官方文档：https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### 第 2 步：体检

```bash
bash setup/check_env.sh
```

会逐项打印 `[ OK ]` / `[FAIL]`。第一次跑大部分是 FAIL，正常。

### 第 3 步：一件安装（约 5 分钟）

```bash
bash setup/install_deps.sh
```

它做 5 件事：

1. 检查环境（Ubuntu 24.04 / Jazzy / colcon / git）
2. `apt install` 依赖：`ros-jazzy-perception-pcl`、`ros-jazzy-vision-opencv`、
   `ros-jazzy-ros-gz`（Gazebo Harmonic）、`libgtsam-dev`（GTSAM 4.2）、
   `libceres-dev` + `libgoogle-glog-dev`（A-LOAM 用）等
3. `git clone` **两套**算法：LIO-SAM 切到 `ros2` 分支；A-LOAM-ROS2 钉在固定 commit
4. 打补丁：LIO-SAM 5 处、A-LOAM 3 处（原因见 [第 8 节](#8-为了跑通我做了什么)）
5. `colcon build`（两套一起编）

编译完再 source 一次：

```bash
source ros2_ws/install/setup.bash
```

> 💡 把它写进 `~/.bashrc` 少打很多字：
> ```bash
> echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
> echo "source ~/AAE4203_LIOSAM_demo/ros2_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## 4. Demo 1：只看 Gazebo + RViz 的传感器

**先跑这个**，确认 Gazebo 和桥接没问题，再上 LIO-SAM。不然出错你分不清是谁的问题。

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_DEMO/ros2_ws/install/setup.bash
ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py
```

你会看到：Gazebo 窗口里出现一个蓝色小车 → 自己开动绕圈；RViz 里是 360° 点云在转。

**另开一个终端检查话题**（新终端记得重新 `source` 两次）：

```bash
ros2 topic list
ros2 topic hz /points          # 应该 ≈ 10 Hz
ros2 topic hz /imu/data        # 应该 ≈ 200 Hz
ros2 topic hz /clock           # 应该 ≈ 1000 Hz（仿真时钟）
ros2 topic echo /imu/data --once
ros2 topic echo /points --once --field fields   # 应该能看到 x,y,z,intensity,ring,time
```

常用参数：

```bash
# 不开 RViz（SSH / 无显示器）
ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py rviz:=false

# 手动控制（另开终端 ros2 run teleop_twist_keyboard teleop_twist_keyboard）
ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py drive:=teleop

# 车不动
ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py drive:=none
```

> Demo 1 里 RViz 的 `Fixed Frame` 是 `lidar_link`，因为这时还没有任何 TF 树。
> 你看到的是**雷达自己坐标系里的点云**，所以车转弯时点云会跟着转 —— 这是正常的。

---

## 5. Demo 2：跑 LIO-SAM 建图

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_DEMO/ros2_ws/install/setup.bash
ros2 launch lio_sam_gz_bringup lio_sam_gazebo.launch.py
```

启动的东西 = Demo 1 的全部 + 3 个静态 TF + LIO-SAM 的 4 个节点 + 建图版 RViz。

**等 10 秒左右**，RViz 里应该开始出现地图。跑 30 秒左右地图基本成型。

### 确认 LIO-SAM 真的在工作

```bash
# 1) 4 个节点都在
ros2 node list
#   /lio_sam_imuPreintegration
#   /lio_sam_imageProjection
#   /lio_sam_featureExtraction
#   /lio_sam_mapOptimization
#   /lio_sam_transformFusion

# 2) 里程计有输出，而且频率不低（IMU 频率）
ros2 topic hz /lio_sam/odometry/imu

# 3) 地图在涨
ros2 topic hz /lio_sam/mapping/map_global      # ≈ 0.2 Hz

# 4) TF 树是否连通（重点看 map→odom→base_link→lidar_link）
ros2 run tf2_tools view_frames
# 生成 frames.pdf，打开看
ros2 run tf2_ros tf2_echo map base_link
```

### 存下这张地图

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.1, destination: /maps/lab1_room}"
# 会存到 ~/maps/lab1_room/{GlobalMap.pcd, CornerMap.pcd, SurfMap.pcd, trajectory.pcd}
```

看 pcd：`pcl_viewer ~/maps/lab1_room/GlobalMap.pcd`（`sudo apt install pcl-tools`）

### 录个 bag 反复复现（做报告很有用）

```bash
ros2 bag record -o lab1_run /points /imu/data /clock /tf /tf_static /cmd_vel
# 玩够了 Ctrl-C，然后离线重放：
ros2 launch lio_sam_gz_bringup lio_sam_gazebo.launch.py drive:=none &
ros2 bag play lab1_run --clock -r 1.0
```

---

---

## 5.5 Demo 3：跑 LOAM baseline

同一个 Gazebo 场景、同一个 16 线雷达，把点云喂给 **A-LOAM**（LOAM 的 ROS 2 移植，上游
[`G-Apple1/A-LOAM-ROS2`](https://github.com/G-Apple1/A-LOAM-ROS2)），作为对照实验的 baseline。
参数已经按"和 LIO-SAM 逐项对齐"设好（对照表在 launch 文件开头）。

```bash
ros2 launch lio_sam_gz_bringup loam_gazebo.launch.py
```

RViz 里应该看到：**橙色**全局地图 `/laser_cloud_map`、当前帧 `/velodyne_cloud_registered`、
**黄色**轨迹 `/aft_mapped_path`。

轨迹应该也是一个圆，但**比 LIO-SAM 抖** —— 因为它没有 IMU 去畸变，
而 Gazebo 的雷达是"一帧瞬时扫完"的、车却在动。这个差异正是报告要量的东西。

确认它真的在算：

```bash
ros2 topic hz /aft_mapped_to_init      # ≈ 10 Hz，scan-to-map 之后的位姿（评测用这个）
ros2 topic hz /laser_odom_to_init      # ≈ 10 Hz，scan-to-scan 里程计
ros2 topic echo /aft_mapped_to_init --once
```

| LOAM 侧参数 | 本 demo 取值 | 对应 LIO-SAM 参数 |
|---|---|---|
| `scan_line` | 16 | `N_SCAN: 16` |
| `minimum_range` | 0.3 m | `lidarMinRange: 0.3` |
| `mapping_line_resolution` | 0.1 m | `mappingCornerLeafSize: 0.1` |
| `mapping_plane_resolution` | 0.2 m | `mappingSurfLeafSize: 0.2` |
| IMU / 去畸变 / 回环 | ✗ / ✗ / ✗ | ✓ / ✓ / ✗（两边都关） |

> 上游代码在 Ubuntu 24.04 上编不过（Ceres 2.2 删掉了 `LocalParameterization`），
> `setup/patch_aloam.sh` 会把它换成 `Manifold`，**不碰算法任何一行**。
> 补丁细节见 [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md)。

---

## 5.6 Demo 4 + 对比实验：谁更精确，差多少

### (a) 同屏看（演示 / 录视频用）

```bash
ros2 launch lio_sam_bench compare_gazebo.launch.py
```

一次 Gazebo，两条算法同时跑，同一个 RViz 里叠三条轨迹：
**蓝色 = LIO-SAM**、**橙色 = LOAM**、**白色 = 真值**。
（Jetson 上会比较吃力，这个 launch 的意义是"肉眼对比"，不是出数据。）

### (b) 正式出数据（报告 / PPT 用这个）

```bash
bash experiments/run_bench.sh --list                    # 看有哪些场景
bash experiments/run_bench.sh --scenarios s0_nominal    # 先验一组（约 4 分钟）
bash experiments/run_bench.sh                           # 全量 S0–S5（约 45 分钟）
cat results/tables/all_runs.md                          # ★ 主表
```

流程：**录一次包 → 两条算法各回放一次 → 统一评测**。
所以两条算法看到的点云流逐字节相同、对齐到的真值逐帧相同，
差异只能来自算法本身。协议、指标定义、6 个场景、排错表全在
👉 **[`experiments/README.md`](experiments/README.md)**

产出：

```
results/raw/<场景>/<算法>.csv     逐帧「算法位姿 + 真值」（可直接 pandas 读）
results/tables/all_runs.md        主表：ATE / RPE / 终点误差 / 鲁棒性 / 输出频率
results/json/<场景>.json          全部标量指标（可追溯）
results/figures/*.png             轨迹图、误差曲线、ATE 柱状图
```

---

## 6. 数据流（谁发谁收）

```
┌─────────────────────────── Gazebo Harmonic (gz sim) ───────────────────────────┐
│  worlds/lab1_room.sdf  +  models/lio_bot/model.sdf                             │
│      gpu_lidar ──► /points_raw   (gz.msgs.PointCloudPacked)                    │
│      imu       ──► /imu_raw      (gz.msgs.IMU)                                 │
│      DiffDrive ◄── /cmd_vel       (gz.msgs.Twist)                              │
│      /clock                                                                     │
└───────────────────────────────────┬─────────────────────────────────────────────┘
                                    │  ros_gz_bridge（config/gz_bridge.yaml）
                                    ▼
              /points_raw  /imu_raw  /clock  /cmd_vel   ←—— 都是标准 ROS 2 消息
                                    │
                    ┌───────────────┴────────────────┐
                    │  sensor_prep.py（★关键胶水）    │
                    │   · 点云补 ring / time 字段     │
                    │   · 点云 frame_id → lidar_link  │
                    │   · IMU frame_id → imu_link     │
                    │   · IMU 没姿态时用重力补 roll/pitch │
                    └───────┬───────────────┬─────────┘
                            │ /points       │ /imu/data
                            ▼               ▼
                    ┌───────────────────────────────────────┐
                    │            LIO-SAM（4 个节点）          │
                    │  imageProjection → featureExtraction   │
                    │        ↓                     ↓         │
                    │   imuPreintegration  ←→  mapOptimization│
                    └───────────────────────────────────────┘
                            │ /lio_sam/mapping/{odometry, path,
                            │                     map_global, cloud_registered}
                            ▼
                         RViz2
```

---

## 7. 手动逐个终端启动（教学用）

想看每一步在干什么，就把 `gazebo_lab1.launch.py` 拆成 6 个终端（每个终端都要先
`source /opt/ros/jazzy/setup.bash && source $ROS_DEMO/ros2_ws/install/setup.bash`）：

```bash
PKG=$(ros2 pkg prefix lio_sam_gz_bringup)/share/lio_sam_gz_bringup

# T1  Gazebo
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r -v 3 $PKG/worlds/lab1_room.sdf"

# T2  把小车放进场景
ros2 run ros_gz_sim create -world lab1_room -name lio_bot \
        -file $PKG/models/lio_bot/model.sdf -x 2.5 -y 0.0 -z 0.02

# T3  桥接
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$PKG/config/gz_bridge.yaml

# T4  点云/IMU 格式转换
ros2 run lio_sam_gz_bringup sensor_prep.py --ros-args -p use_sim_time:=true

# T5  让它动起来
ros2 run lio_sam_gz_bringup patrol_driver.py --ros-args -p use_sim_time:=true

# T6  三个静态 TF（LIO-SAM 只发 odom→lidar_link 和 odom→base_link）
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
     --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.28 \
     --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id lidar_link
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.28 \
     --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id imu_link

# T7  LIO-SAM 四个节点
LIO_ARGS="--ros-args --params-file $PKG/config/params_gazebo.yaml -p use_sim_time:=true"
ros2 run lio_sam lio_sam_imuPreintegration $LIO_ARGS
ros2 run lio_sam lio_sam_imageProjection   $LIO_ARGS
ros2 run lio_sam lio_sam_featureExtraction $LIO_ARGS
ros2 run lio_sam lio_sam_mapOptimization   $LIO_ARGS

# T8  RViz
rviz2 -d $PKG/rviz/lio_sam_slam.rviz --ros-args -p use_sim_time:=true
```

**这一步是理解系统的最好方式**：每开一个终端，就用 `ros2 topic list` 看多了哪些话题，
用 `rqt_graph` 看节点连起来没有。

---

## 8. 为了跑通我做了什么

官方 LIO-SAM 是给"真实雷达 + 真实 IMU + ROS 1/早期 ROS 2"写的。要跑在
Gazebo + Jazzy + GTSAM 4.2 上，只有 4 处需要处理，全部已经做好了：

### 8.1 `sensor_prep.py`：补 `ring` 和 `time`（唯一的新代码）

LIO-SAM 把点云当 Velodyne 的 `PointXYZIRT` 处理（`imageProjection.cpp` 开头）：

```cpp
struct VelodynePointXYZIRT {
    float x, y, z, intensity;
    uint16_t ring;   // 第几线 —— 用来把点排进 range image 的行
    float    time;   // 相对本帧起始时刻的偏移 —— 用来去畸变
};
```

而 Gazebo 的 `gpu_lidar` 只能给 `x,y,z,intensity`。所以 `sensor_prep.py`：

- **ring**：由点的垂直角反算。我们设的是 16 线、垂直视场 ±15°、每线 2°，所以
  `ring = round((仰角 + 15°) / 2°)`，正好和 `imageProjection.cpp` 里
  `rowIdn = (verticalAngle + (N_SCAN-1)) / 2.0` 这个兜底公式一致。
- **time**：默认全部填 0。**这是有道理的**：Gazebo 的激光雷达不是真的在转，
  一帧就是一瞬间扫完的，本来就没有 0.1 s 的扫描过程，所以"不去畸变"才是对的。
  想体验真实驱动的时间分配，把参数 `time_period` 设成 `0.1` 即可。
- 顺便把 `frame_id` 强制写成 `lidar_link` / `imu_link`，因为 gz 发出来的
  `frame_id` 是 `lio_bot/lidar_link/lidar` 这种内部名字。
- IMU 的 `orientation` 如果是全 0（有些仿真器这样），用加速度计的重力方向补一个
  roll/pitch —— 否则 LIO-SAM 会直接 `Invalid quaternion, please use a 9-axis IMU!` 退出。

### 8.2 三个编译补丁（`setup/patch_lio_sam.sh`）

| 位置 | 原文 | 改成 | 原因 |
|---|---|---|---|
| `imuPreintegration.cpp` | `#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>` | 注释掉 | Ubuntu 的 `libgtsam-dev` (4.2.0) **不带 `gtsam_unstable` 模块**；而且这个头文件里的类在 LIO-SAM 里根本没被用到 |
| `imuPreintegration.cpp` | `boost::shared_ptr<gtsam::PreintegrationParams> p = ...MakeSharedU(...)` | `auto p = ...` | GTSAM 从 4.2 起把 `boost::shared_ptr` 换成了 `std::shared_ptr`，用 `auto` 接就与版本无关了 |
| `CMakeLists.txt` | `set(OpenCV_STATIC ON)` | 注释掉 | Ubuntu 的 apt 版 OpenCV 只有 `.so`，强制静态链接会找不到库 |

补丁是**幂等**的，重复跑没问题；想撤销就 `git -C ros2_ws/src/LIO-SAM checkout .`。

### 8.3 参数（`config/params_gazebo.yaml`）

和官方 `params.yaml` 的差异，每一条都必须改：

| 参数 | 官方 | 本 demo | 为什么 |
|---|---|---|---|
| `sensor` | `ouster` | `velodyne` | 我们按 Velodyne 的字段格式打包点云 |
| `N_SCAN` | 64 | **16** | `model.sdf` 里 `vertical/samples = 16` |
| `Horizon_SCAN` | 512 | **360** | `model.sdf` 里 `horizontal/samples = 360` |
| `extrinsicRot` / `extrinsicRPY` | 非单位阵 | **单位阵** | 仿真里 `lidar_link` 和 `imu_link` 同位置同朝向 |
| `extrinsicTrans` | 有偏移 | `[0,0,0]` | 同上 |
| `lidarMinRange / MaxRange` | 1.0 / 1000 | **0.3 / 40** | 室内 12×12 m 场景 |
| `loopClosureEnableFlag` | true | **false** | 先跑通；回环 ICP 比较慢，见[练习](#10-练习) |

### 8.4 静态 TF

读了 `ros2` 分支的源码后确认：LIO-SAM **只发两个 TF**——

- `imuPreintegration.cpp` 的 `TransformFusion`：`odom → base_link`
- `mapOptmization.cpp` 的 `publishOdometry()`：`odom → lidar_link`

**`map → odom` 要自己发**（官方 `run.launch.py` 里也是一个
`static_transform_publisher ... map odom`），`base_link → lidar_link` 也要自己发。
这就是 `lio_sam_gazebo.launch.py` 里那 3 个静态 TF 的来历。

---

## 9. 排错 FAQ

> 出问题先做这件事：**看终端里是谁在报错**。RViz 报错≠LIO-SAM 报错≠Gazebo 报错。

| 现象 | 大概率原因 | 处理 |
|---|---|---|
| Gazebo 黑屏 / 一开就崩 | 显卡驱动 / WSLg 渲染 | `export LIBGL_ALWAYS_SOFTWARE=1` 再试；或 `gz sim -v 4 <world>` 看详细日志 |
| Gazebo 起来了但没有小车 | `create` 抢跑 / 服务名不对 | 看终端有没有 `Waiting for service [/world/lab1_room/create]`；手动跑第 7 节 T2 |
| `ros2 topic list` 里没有 `/points_raw` | 桥接没起 / 话题名不一致 | `ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=...` 看有没有报 "cannot create bridge" |
| `/points` 有但 `/imu/data` 没有 | `sensor_prep.py` 没起来 | `ros2 node list \| grep sensor_prep` |
| RViz 里点云一直 "Status: Ok" 但看不到 | QoS 不匹配 / Fixed Frame 不对 | 点 Display → `Reliability Policy` 改成 **Best Effort**；Fixed Frame 换成 `lidar_link` |
| RViz 打开是空白 / 报 config 解析失败 | `.rviz` 版本差异 | 直接 `rviz2` 手加 Display：Fixed Frame `map` → 加 PointCloud2 指向 `/lio_sam/mapping/map_global` 和 `/lio_sam/mapping/cloud_registered`，再加 Path 指向 `/lio_sam/mapping/path` |
| LIO-SAM 一直打印 `Waiting for IMU data ...` | IMU 频率太低 / 时间戳没覆盖 | `ros2 topic hz /imu/data`，要 ≥200 Hz；确认所有节点 `use_sim_time:=true` |
| `deskew function disabled, system will drift significantly` | 点云没有 `time` 字段 | 你大概没走 `sensor_prep.py` |
| `Point cloud ring channel not available` | 点云没有 `ring` 字段 | 同上 |
| `Invalid quaternion, please use a 9-axis IMU!` | IMU 没姿态 | 用 `sensor_prep.py`；它会在 orientation 全 0 时补 roll/pitch |
| `Not enough features! Only 3 edge and 20 planar features available.` | 特征太少：场景太空 / 参数不匹配 | 确认 `N_SCAN`=16、`Horizon_SCAN`=360；把车开到有墙有柱子的地方 |
| 地图/轨迹"炸开"、乱跳 | 外参或重力方向错 | 检查 `extrinsicRot`；确认 world 的 `<gravity>` 和 `imuGravity` 都是 9.80511 |
| 轨迹是一团乱麻 | `use_sim_time` 没统一 | 所有节点（含 rviz2）都要 `use_sim_time:=true` |
| TF 报 `Lookup would require extrapolation` | 时钟不同步 / 静态 TF 没起 | `ros2 run tf2_tools view_frames` 看树断在哪 |
| 编译报 `find_package(GTSAM)` | 没装 GTSAM | `sudo apt install libgtsam-dev` |
| 编译报 `gtsam_unstable/...` | 没打补丁 | `bash setup/patch_lio_sam.sh` |
| 编译报 `boost::shared_ptr` 相关 | 没打补丁 | 同上 |
| `ros2 run` 报 `Permission denied` | .py 没有可执行位（用了 `--symlink-install`） | `chmod +x ros2_ws/src/lio_sam_gz_bringup/scripts/*.py` 后重新 build |
| 编译在 `mapOptmization.cpp` 卡很久 / OOM | 并行编译内存不够 | `colcon build --executor sequential` 或 `--parallel-workers 1` |

### 9.1 如果想一步步调试

出问题时按这个顺序二分，能快速定位是"仿真/桥接"的锅还是"LIO-SAM"的锅：

```bash
# ① Gazebo 里能看到车、车在动？           → 不行 = world / model / physics 的问题
# ② ros2 topic hz /points_raw 有 10 Hz？   → 不行 = 桥接或 create 的问题
# ③ ros2 topic echo /points --once --field fields  看到 ring/time？ → 不行 = sensor_prep 的问题
# ④ ros2 topic hz /lio_sam/odometry/imu 有 200 Hz？ → 不行 = IMU 或 imuPreintegration 的问题
# ⑤ ros2 topic hz /lio_sam/deskew/cloud_info 有 10 Hz？ → 不行 = 点云格式或 N_SCAN/Horizon_SCAN 不匹配
# ⑥ ros2 topic hz /lio_sam/feature/cloud_info 有 10 Hz？ → 不行 = 特征太少（看 "Not enough features" 日志）
# ⑦ /lio_sam/mapping/map_global 有没有点？  → 有 = 成了
```

---

## 10. 练习

按难度排的，做完基本就把 LIO-SAM 摸清了：

1. **【10 分钟】改场景**：在 `worlds/lab1_room.sdf` 里再加两个柱子，重新 `colcon build`，
   看地图里有没有它们。
2. **【10 分钟】改路线**：`patrol_driver.py` 的 `linear_speed` / `angular_speed` 改一下，
   比如 `0.6 / 0.6`（半径 1 m 更快），看轨迹还圆不圆。
3. **【20 分钟】打开回环**：`params_gazebo.yaml` 里 `loopClosureEnableFlag: true`，
   让它转 2 圈以上，观察 `map_global` 有没有被"拉回来"。注意终端里的
   `ICP has not converged` 提示 —— 那是正常的。
4. **【20 分钟】体验去畸变**：`sensor_prep.py` 的 `time_period` 参数设成 `0.1`
   （`ros2 run lio_sam_gz_bringup sensor_prep.py --ros-args -p time_period:=0.1`），
   把车速调快，对比有没有 time 字段时地图的差别。
5. **【30 分钟】故意搞坏**：把 `extrinsicRot` 第 3 行改成 `0,0,-1`（z 轴翻转），
   重启，观察"上下乱跳"。**这个实验比读 10 篇博客有用**。
6. **【1 小时】加 GPS 因子**：`model.sdf` 里加 `type="navsat"` 传感器，桥接成
   `sensor_msgs/NavSatFix`，用 `robot_localization` 的 `navsat_transform_node`
   转成 `nav_msgs/Odometry`，再把 `gpsTopic` 指过去。
7. **【2 小时】评估精度**：把 `/lio_sam/mapping/odometry` 和 Gazebo 的 `/odom_gz`
   都录进 bag，用 Python 对齐时间戳、算 ATE/RPE，画 XY 对比图。
8. **【进阶】换成真实数据**：下 KITTI 00 序列，按 LIO-SAM 官方 README 改
   `N_SCAN: 64`、`downsampleRate: 2`、`extrinsicTrans`，用 `ros2 bag play` 跑。

---

## 11. 命令速查

```bash
# ---- 每次开新终端 ----
source /opt/ros/jazzy/setup.bash
source $ROS_DEMO/ros2_ws/install/setup.bash

# ---- 跑 ----
ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py       # Demo 1
ros2 launch lio_sam_gz_bringup lio_sam_gazebo.launch.py    # Demo 2

# ---- 看 ----
ros2 node list                                  # 有哪些节点
ros2 topic list                                 # 有哪些话题
ros2 topic hz /points                           # 频率
ros2 topic echo /input --once                   # 内容
ros2 run rqt_graph rqt_graph                    # 节点连接图（最直观）
ros2 run rqt_tf_tree rqt_tf_tree                # TF 树
ros2 run tf2_tools view_frames                  # 导出 frames.pdf
ros2 run tf2_ros tf2_echo map base_link         # 某一对 TF 的实时值
ros2 param list /lio_sam_mapOptimization        # 参数
ros2 param get /lio_sam_mapOptimization N_SCAN

# ---- 存图 / 录包 ----
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.1, destination: /maps/lab1}"
ros2 bag record -o lab1_run /points /imu/data /clock /tf /tf_static
ros2 bag play lab1_run --clock
ros2 bag info lab1_run

# ---- 清理重来 ----
rm -rf $ROS_DEMO/ros2_ws/{build,install,log}
cd $ROS_DEMO/ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 附：验证状态（**已实测通过**）

本 demo 已在下面这套**真实环境**上端到端跑通：

| 项目 | 实测值 |
|---|---|
| 硬件 | NVIDIA **Jetson Orin Nano**（aarch64，8 GB 统一内存） |
| 系统 / ROS | Ubuntu **24.04.5 LTS** + **ROS 2 Jazzy** |
| 仿真 | Gazebo **Harmonic**（`gz-sim 8.15.0`，经 `ros-jazzy-ros-gz`） |
| 数学库 | GTSAM **4.2.0**（Ubuntu 官方 `libgtsam-dev`，**无需编译**）、Eigen 3.4、PCL 1.14 |
| `/points` | **≈ 7 Hz**（仿真 10 Hz × 实时因子 0.70） |
| `/imu/data` | **≈ 140 Hz**（仿真 200 Hz × 0.70） |
| LIO-SAM | 4 个节点全部启动；RViz `Global Status: Ok`；轨迹为闭合圆；`map_global` 出墙面 |
| 编译产物 | `install/lio_sam/lib/lio_sam/` 下 4 个可执行文件齐全 |

判据很直观：**`140/200 = 7/10 = 0.70`** —— 两个频率比值一致，说明传感器都准，只是仿真跑不到实时。

### 实战中遇到了 18 个问题（16 个需要动手修）

Ubuntu `libgtsam-dev` 打包缺陷 3 处、GTSAM 4.2 API 差异 2 处、Eigen 相关 2 处、
Jetson 内存 OOM、gz 点云话题嵌套、gz IMU 独立系统插件、spawn 落进方柱、SDF 欧拉角顺序写反、
launch 参数撞名、室外默认可视化参数……

完整的**症状 → 根因 → 修复 → 验证**手册（按症状索引）：

> 👉 **[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md)** —— **排错时优先查这份**

其中对**复现**最关键的三条：

1. **8 GB 内存机器必须先加 8 GB swap**，编译时 `CMAKE_BUILD_PARALLEL_LEVEL=1`，
   否则 `cc1plus` 会被 OOM kill（连 VS Code 远程窗口会一起崩，`code: 9`）。
2. **`setup/install_deps.sh` 会自动打 5 处补丁**，别手动跳过：
   `gtsam_unstable` 头文件、`boost::shared_ptr`、`find_package(Eigen)`、
   `ament_target_dependencies(GTSAM Eigen)`、`set(OpenCV_STATIC ON)`；
   另外还要 `apt install libmetis-dev libsuitesparse-dev`（`libgtsam-dev` 的依赖声明不全）。
3. **重启前务必 `pkill -f gz-sim`**。`pkill -f gazebo_lab1` 只杀 launch 脚本，
   `gz sim` 会活下来变成**第二个仿真** → `/clock` 双源 → RViz 狂刷 `jump back in time`、
   实时因子掉一半（实测从 0.70 掉到 0.34）。

不想启动 ROS 也能验证代码本身没坏（**五个脚本，全部不需要 Gazebo**）：

```bash
python3 setup/_selfcheck.py      # 静态校验 128 项：YAML / SDF / launch 字符串 /
                                 #   两套算法的参数是否对齐 / 参数文件是否合法 /
                                 #   有没有 BOM 和 CRLF（Linux 上的 bad interpreter 源头）
python3 setup/_test_pack.py      # 单元测试：ring/time 计算 + PointCloud2 打包布局
python3 setup/_test_metrics.py   # 评测器自检：完美轨迹→1e-16、2% 尺度→25 mm、发散能识别
python3 setup/_test_pipeline.py  # 报告链自检：CSV → 指标表 → 图 → PPT 全跑一遍
python3 setup/_test_nodes.py     # gt_sync / traj_recorder 的逻辑自检（stub 掉 rclpy 后真跑）
```

后三个是本项目新增的：**报告里的每个数字都出自这台评测器**，
所以它自己必须先被验证过。具体说：

- `_test_metrics.py` 用解析算例卡指标（2% 尺度误差 → 恰好 25 mm）
- `_test_pipeline.py` 检查 PPT 是否会照实写 "LIO-SAM worse"（防止结论被写死）
- `_test_nodes.py` 用最小 rclpy 桩把真值插值（线性 + slerp）、雷达安装偏移的旋转、
  CSV 列与 meta 落盘都跑一遍 —— 这两个节点每次实验都会跑，但只有在这里被真正执行过

如果哪一步卡住了，把**终端里第一段红色报错**贴出来，配合第 9 节的分步排查（①~⑦）定位最快。

---

## 参考

- LIO-SAM 论文：https://arxiv.org/abs/2007.00258
- LIO-SAM 代码：https://github.com/TixiaoShan/LIO-SAM （`ros2` 分支）
- ROS 2 Jazzy：https://docs.ros.org/en/jazzy/
- Gazebo Harmonic：https://gazebosim.org/docs/harmonic/
- `ros_gz`（bridge/sim）：https://github.com/gazebosim/ros_gz
- `ros_gz_sim_demos`（官方小 demo，排查环境必备）：https://docs.ros.org/en/jazzy/p/ros_gz_sim_demos/
- GTSAM：https://gtsam.org/get_started/
- 完整参考清单见 [`docs/LIO-SAM-intro-and-roadmap.md` 第 10 节](docs/LIO-SAM-intro-and-roadmap.md#10-参考文档)
