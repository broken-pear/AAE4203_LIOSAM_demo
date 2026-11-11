# 排错手册：LIO-SAM on ROS 2 Jazzy × Gazebo Harmonic × Jetson Orin Nano

> 这份文档记录的是**真实踩过的 18 个问题（16 个需要动手修 + 2 个只需知道的无害告警）**，全部发生在下面这套具体环境里，每一个都包含
> **症状 / 根因 / 修复 / 验证**。按"症状"查就行（见快速索引）。
>
> 如果你想重现这套 demo 而不是排错，请读 [`README.md`](../README.md)。

---

## 环境基线（实测值）

| 项目 | 值 |
|---|---|
| 硬件 | NVIDIA **Jetson Orin Nano**（aarch64，8 GB 统一内存） |
| 系统 | Ubuntu **24.04.5 LTS**（`ports.ubuntu.com` arm64 源） |
| ROS | **ROS 2 Jazzy**（`ros-jazzy-desktop`） |
| 仿真 | Gazebo **Harmonic / gz-sim 8.15.0**（由 `ros-jazzy-ros-gz` 的 vendor 包提供） |
| 数学库 | GTSAM **4.2.0**（Ubuntu 官方 `libgtsam-dev`）、Eigen 3.4、PCL 1.14 |
| SLAM | LIO-SAM `ros2` 分支 |
| 实测频率 | `/points` ≈ **7 Hz**（仿真 10 Hz × RTF 0.70）· `/imu/data` ≈ **140 Hz**（仿真 200 Hz × 0.70） |
| 实测实时因子 | **≈ 0.70**（`140/200 = 7/10 = 0.70`，两个比值一致即说明传感器本身准） |

---

## 快速索引（按症状查）

| 症状 | 章节 |
|---|---|
| `apt install ./x.deb` 报所有依赖 "not installable" | [A1](#a1-apt-install-xdeb-报所有依赖-not-installable) |
| 编译到一半 `cc1plus` 被 kill / VS Code 窗口崩溃 `code: 9` | [A2](#a2-编译到一半进程被-kill--vs-code-远程窗口崩溃) |
| `The imported target "CppUnitLite" references the file ... does not exist` | [B1](#b1-gtsam-的-cppunitlite-静态库不存在) |
| `gtsam_unstable/.../IncrementalFixedLagSmoother.h: No such file` | [B2](#b2-gtsam_unstable-头文件找不到) |
| `boost::shared_ptr<gtsam::PreintegrationParams>` 编译报错 | [B3](#b3-boostshared_ptr-与-gtsam-42-不兼容) |
| `find_package(Eigen)` 找不到 `EigenConfig.cmake` | [B4](#b4-find_packageeigen-失败) |
| `ament_target_dependencies() ... 'Eigen' was not found before` | [B5](#b5-ament_target_dependencies-报-eigen-未找到) |
| `No rule to make target '.../libmetis.so'` | [B6](#b6-libmetisso-缺失) |
| `cannot find -lccolamd` | [B7](#b7-cannot-find-lccolamd) |
| **话题存在但没有数据**（`/points_raw`） | [C1](#c1-点云话题存在但没数据) |
| **话题存在但没有数据**（`/imu_raw`） | [C2](#c2-imu-话题存在但没数据) |
| Gazebo 里看不到小车 | [C3](#c3-gazebo-里看不到小车) |
| 轮子在"翻滚"而不是滚动 | [C4](#c4-轮子在翻滚而不是滚动) |
| `libEGL warning: egl: failed to create dri2 screen` | [C5](#c5-libegl-warning) |
| Demo 2 的 RViz 根本不启动 | [D1](#d1-demo-2-的-rviz-不启动) |
| RViz 里地图只有几段弧 | [D2](#d2-rviz-里地图只有几段弧) |

---

## A. 环境与资源

### A1. `apt install ./x.deb` 报所有依赖 "not installable"

**症状**

```
The following packages have unmet dependencies:
 code:amd64 : Depends: libc6:amd64 (>= 2.15) but it is not installable
              Depends: libglib2.0-0:amd64 (>= 2.39.4) but it is not installable
              ...
```

连 `libc6 >= 2.15` 这种必然满足的依赖都被判"不可安装"。

**根因**：下载的 deb **架构不对**。Jetson 是 `aarch64`，源里只有 arm64 包，amd64 的依赖当然一个都找不到。（如果是 apt 索引坏了，`libc6` 本身仍是已安装状态，不会出现在未满足列表里。）

**修复**

```bash
uname -m                      # aarch64 ?
dpkg --print-architecture     # arm64 ?
apt-cache policy libc6        # Candidate 有值 = apt 索引是健康的，别去重建源
```

换成 arm64 包：

```bash
wget "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-arm64" -O code-arm64.deb
dpkg -I code-arm64.deb | grep -i architecture     # 必须显示 arm64
sudo apt install ./code-arm64.deb
```

**顺带解释那条无害警告**：

```
N: Download is performed unsandboxed as root as file '...' couldn't be accessed by user '_apt'
```

本地 deb 在 `~/` 下、`_apt` 用户读不到，apt 退回用 root 直接读。**不是错误**，装是成功的。

---

### A2. 编译到一半进程被 kill / VS Code 远程窗口崩溃

**症状**

- `c++: fatal error: Killed signal terminated program cc1plus`
- 或者 VS Code 弹窗 `The window terminated unexpectedly (reason: 'killed', code: '9')`

```bash
sudo dmesg -T | grep -iE 'out of memory|killed process' | tail
# [Out of memory: Killed process 41837 (code) ... oom_score_adj:300]
# [Out of memory: Killed process 50585 (cc1plus) ...]
```

**根因**：Orin Nano 8 GB 共享内存，编译 `mapOptmization.cpp`（77 KB 源码 + GTSAM 重模板）时
单个 `cc1plus` 峰值 1 GB 以上；再开 2 并发 + VS Code Server（`oom_score_adj:300`，**首选牺牲品**）
+ C/C++ 扩展索引 → 必然 OOM。而且 **`Swap` 默认是 0**。

**修复（三件套）**

```bash
# ① 加 8 GB swap（最关键，后面跑 Gazebo 也靠它）
df -h / | tail -1                    # 先确认磁盘够
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
free -h                              # Swap 应为 8.0Gi

# ② 关掉 VS Code（省 1~2 GB），用独立 GNOME Terminal / ssh

# ③ 单线程 + 降优化等级
cd ~/AAE4203/AAE4203_LIOSAM_demo/ros2_ws
rm -rf build/lio_sam
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG" \
  2>&1 | tee ~/build.log
```

**为什么降 `-O2`**：`-O3` 的激进内联让编译内存高 30~40%；LIO-SAM 里重的是 PCL/GTSAM 模板代码，
`-O3` 对运行速度几乎无收益。

**注意**：重编时 **cmake 参数必须和上次完全一致**，否则 CMake 认为选项变了会全量重编（又要 3 分钟）。

---

## B. 依赖与编译（LIO-SAM）

> B1 / B6 / B7 是**同一个根因**：Ubuntu 的 `libgtsam-dev` 包**依赖声明不完整**。
> 这条在 amd64 上同样存在，不是 arm64 特有。

### B1. GTSAM 的 CppUnitLite 静态库不存在

**症状**

```
CMake Error at /usr/lib/cmake/GTSAM/GTSAM-exports.cmake:110 (message):
  The imported target "CppUnitLite" references the file
    "/usr/lib/aarch64-linux-gnu/libCppUnitLite.a"
  but this file does not exist.
```

**根因**：`GTSAM-exports.cmake` 声明了 `CppUnitLite`（GTSAM 自己的单元测试框架），但打包时
**没把 `.a` 装进去**。LIO-SAM 从头到尾不 include、也不链接它（只链 `gtsam`）。

**修复**（造一个空归档骗过 CMake 的存在性检查）

```bash
bash setup/fix_gtsam_cmake.sh
# 或手写：
sudo ar rcs /usr/lib/aarch64-linux-gnu/libCppUnitLite.a
```

**坑中坑**：`GTSAM-exports.cmake` 里的路径是 `${_IMPORT_PREFIX}/<multiarch>/libXxx.a`，
所以**用 grep 搜字面 `/usr/lib/...` 是搜不到的**，必须先求出 multiarch 三元组
（`gcc -dumpmachine` → `aarch64-linux-gnu`）。而且 per-config 的 `IMPORTED_LOCATION` 在
`GTSAM-exports-<config>.cmake` 里（Ubuntu 上是 `GTSAM-exports-relwithdebinfo.cmake`），
扫描要覆盖 `GTSAM-exports*.cmake`。

**验证**

```bash
ls -l /usr/lib/aarch64-linux-gnu/libCppUnitLite.a     # 8 字节，空归档
cat /usr/lib/cmake/GTSAM/GTSAM-exports*.cmake | grep -oE 'lib[A-Za-z0-9_+.-]*\.a' | sort -u
# 正常只有 libCppUnitLite.a 一个
```

---

### B2. gtsam_unstable 头文件找不到

**症状**

```
fatal error: gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h: No such file or directory
```

**根因**：Ubuntu 的 `libgtsam-dev` **裁掉了 `gtsam_unstable` 模块**；而 LIO-SAM 的
`imuPreintegration.cpp` include 了它，却**从来没用到** `IncrementalFixedLagSmoother`。

**修复**

```bash
sed -i 's@^#include <gtsam_unstable.*$@// [patched] Ubuntu libgtsam-dev 不含该模块@' \
  ros2_ws/src/LIO-SAM/src/imuPreintegration.cpp
```

---

### B3. boost::shared_ptr 与 GTSAM 4.2 不兼容

**症状**

```
error: cannot convert 'std::shared_ptr<gtsam::PreintegrationParams>' to
       'boost::shared_ptr<gtsam::PreintegrationParams>'
```

**根因**：GTSAM **4.2 起把智能指针从 `boost::shared_ptr` 换成了 `std::shared_ptr`**。

**修复**（用 `auto` 接，4.0~4.2 通吃）

```bash
sed -i 's@boost::shared_ptr<gtsam::PreintegrationParams> p =@auto p =@' \
  ros2_ws/src/LIO-SAM/src/imuPreintegration.cpp
```

---

### B4. find_package(Eigen) 失败

**症状**

```
By not providing "FindEigen.cmake" in CMAKE_MODULE_PATH this project has
asked CMake to find a package configuration file provided by "Eigen"
Could not find a package configuration file provided by "Eigen"
```

**根因**：Eigen3 只安装 `Eigen3Config.cmake`，**没有 `EigenConfig.cmake`**，CMake 也不自带
`FindEigen.cmake`。这行是 LIO-SAM 的历史遗留。

**修复**

```bash
sed -i 's@^find_package(Eigen REQUIRED)@find_package(Eigen3 REQUIRED)@' \
  ros2_ws/src/LIO-SAM/CMakeLists.txt
```

**验证**：`ls /usr/share/eigen3/cmake/` 应该有 `Eigen3Config.cmake`。

---

### B5. ament_target_dependencies 报 Eigen 未找到

**症状**（通常紧跟着 B4 出现）

```
CMake Error at .../ament_target_dependencies.cmake:77 (message):
  ament_target_dependencies() the passed package name 'Eigen' was not found before
Call Stack: CMakeLists.txt:52 (ament_target_dependencies)
```

**根因**：`ament_target_dependencies()` 会校验每个名字**之前必须被 `find_package` 过**。
B4 把 `Eigen` 改成 `Eigen3` 之后，第 52 行的 `... GTSAM Eigen)` 就失效了。

**修复**

```bash
sed -i 's@GTSAM Eigen)@GTSAM Eigen3)@' ros2_ws/src/LIO-SAM/CMakeLists.txt
grep -n 'GTSAM Eigen' ros2_ws/src/LIO-SAM/CMakeLists.txt    # 只有 1 处
```

> B2~B5 都在 `setup/patch_lio_sam.sh` 和 `setup/patch_eigen.sh` 里自动化了。

---

### B6. libmetis.so 缺失

**症状**

```
gmake[2]: *** No rule to make target '/usr/lib/aarch64-linux-gnu/libmetis.so',
              needed by 'lio_sam_imuPreintegration'. Stop.
```

**根因**：`libgtsam.so` 内部链接 METIS（GTSAM 用它做变量排序），导出的链接接口要求
`libmetis.so` 这个 dev 符号链接；而 `libgtsam-dev` 的 `Depends` **漏了 `libmetis-dev`**，
系统里只有运行时的 `libmetis.so.5`。

**修复**

```bash
sudo apt install -y libmetis-dev
ls -l /usr/lib/aarch64-linux-gnu/libmetis.so
```

---

### B7. cannot find -lccolamd

**症状**

```
/usr/bin/ld: cannot find -lccolamd: No such file or directory
```

**根因**：同上，`libgtsam-dev` 没声明 SuiteSparse 的依赖。`-lccolamd`（短名，无路径）
要求 `libccolamd.so` 符号链接存在。

**修复**（一次补齐整套）

```bash
sudo apt install -y libsuitesparse-dev libtbb-dev
```

`libsuitesparse-dev` 提供 `libcholmod.so` / `libccolamd.so` / `libcolamd.so` / `libamd.so` /
`libcamd.so` / `libspqr.so` / `libumfpack.so` / `libsuitesparseconfig.so`。

**精确检查还缺哪些 `-l`**（不要只查 `ldconfig -p`，那会把 `.so.3` 运行时库误判成"有"）：

```bash
cd ros2_ws
grep -hoE '(^| )-l[^ ]+' build/lio_sam/CMakeFiles/*/link.txt | tr -d ' ' | sort -u \
| while read l; do
    echo 'int main(){return 0;}' | gcc -x c - -o /dev/null "$l" 2>/dev/null || echo "真缺: $l"
  done
```

> ⚠️ 反例：用 `grep -oE 'lib[A-Za-z0-9_+.-]*\.a'` 之类扫 `link.txt` 会产生大量**误报** ——
> `-Wl,-rpath-link` 里的 `-link`、路径 `aarch64-linux-gnu` 里的 `-linux`、截断的
> `-llttng-ust`→`-llttng` 都会被当成库名。**以 linker 的报错为准**。

---

## C. Gazebo 仿真

### C1. 点云话题存在但没数据

**症状**

```bash
ros2 topic list | grep points     # /points_raw  /points 都在
ros2 topic hz /points_raw         # topic [/points_raw] does not appear to be published yet
```

**根因**：**话题存在 ≠ 有数据**。`ros_gz_bridge` 一启动就会 advertise 所有配置过的话题（发布者
存在、一条消息都没发过），`sensor_prep.py` 也会 advertise `/points`。

真正的根因是**桥接订错了 gz 话题**：gz-sim 的 lidar 传感器把数据分成两路发布：

| gz 话题 | 类型 |
|---|---|
| `/points_raw` | `gz.msgs.LaserScan` |
| **`/points_raw/points`** | **`gz.msgs.PointCloudPacked`** ← 我们要的三维点云 |

**诊断（一锤定音）**

```bash
gz topic -l | grep -iE 'points|imu'
timeout 3 gz topic -e -t /points_raw        2>&1 | head -c 300; echo
timeout 3 gz topic -e -t /points_raw/points 2>&1 | head -c 300; echo
```

哪个能 echo 出 `field { name: "x" ... }` 就是对的。

**修复**：`config/gz_bridge.yaml` 里

```yaml
- ros_topic_name: "/points_raw"
  gz_topic_name: "/points_raw/points"        # ← 注意 /points 后缀
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
  frame_id: "lidar_link"
```

**顺带**：gz 发出来的 `frame_id` 是 `lio_bot::lidar_link::lidar` 这种内部名，所以桥接配置里的
`frame_id:` 覆盖（或 `sensor_prep.py` 里强制改）是必须的，否则 LIO-SAM 的 `lidarFrame` 对不上。

---

### C2. IMU 话题存在但没数据

**症状**

```bash
gz topic -l | grep imu     # 只有 /imu_raw 一个（没有嵌套，话题名是对的）
ros2 topic hz /imu/data    # does not appear to be published yet
```

**根因**：**gz-sim 8 的 IMU 是独立系统插件**（`src/systems/imu` 独立成目录），
**不归 `gz-sim-sensors-system` 管**。没加载它，IMU 传感器会被创建、话题会注册，但**永远不发数据**。

**修复**：world 文件里加插件

```xml
<plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-imu-system"               name="gz::sim::systems::Imu"/>          <!-- ← 这一行 -->
<plugin filename="gz-sim-sensors-system"           name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

**验证**

```bash
ros2 topic hz /imu/data          # ≈ 200 Hz（仿真时间）
ros2 topic echo /imu/data --once # orientation 不能是全 0；静止时 linear_acceleration.z ≈ -9.8
```

---

### C3. Gazebo 里看不到小车

**症状**：Gazebo 里有房间、有障碍物，但没有车；`Entity Tree` 里滚不到 `lio_bot`。

**根因**：`ros_gz_sim create` 没给 `-x/-y` 就 spawn 在世界原点 `(0,0)`，而 demo 的
`center_pillar` **正好占着 `(0,0)`（1.2×1.2×2.5 m）** → 车生成在柱子内部，被物理引擎卡住/弹出。

**修复**（`launch/gazebo_lab1.launch.py`）

```python
'-x', '2.5', '-y', '0.0', '-z', '0.02',
```

`(2.5, 0)` 是算过的：`v=0.4, ω=0.32` → 绕半径 1.25 m、圆心 `(2.5, 1.25)` 的圆，
离中央方柱最近 1.54 m、离四个方墩都有 0.5 m 以上余量。

**判断车到底在不在的可靠方法**（比肉眼找靠谱）

```bash
ros2 topic hz /points            # 有数据 = 雷达存在 = 车存在（雷达挂在车上）
ros2 topic echo /odom_gz --once  # 看 pose.position
```

Gazebo GUI 里：`Entity Tree` 滚到底 → `lio_bot` → 右键 `Follow`。

---

### C4. 轮子在"翻滚"而不是滚动

**症状**：小车能动，但轮子看起来像在沿"垂直于自身对称轴"的方向翻跟头，压不出正常的滚动。

**根因**：SDF 的 `<pose>x y z roll pitch yaw</pose>`，圆柱默认轴是 link 的 **z 轴**：

| 旋转 | 圆柱轴 z 变成 | 结果 |
|---|---|---|
| `pitch 90°`（`0 1.5708 0`） | → **x（前后）** | 两个"朝前的圆盘"，轴不在左右方向 |
| `roll 90°`（`1.5708 0 0`） | → **y（左右）** | 正常车轮，轴 = 车轴 |

关节轴写的是 `<xyz>0 1 0</xyz>`（y 方向）→ 与错的几何轴（x）垂直 → 翻滚。

**修复**（`models/lio_bot/model.sdf`，2 个 collision + 2 个 visual 共 4 处）

```xml
<pose>0 0 0 1.5707963 0 0</pose>     <!-- roll 90°，不是 pitch -->
```

```bash
grep -n '1.5707963' ros2_ws/src/lio_sam_gz_bringup/models/lio_bot/model.sdf   # 应为 4 行
```

---

### C5. libEGL warning

**症状**

```
[gazebo-1] libEGL warning: egl: failed to create dri2 screen     ← 出现 4 次
```

**结论**：**这是警告不是错误**。它出现在 OGRE2 初始化阶段，之后 `gpu_lidar` 依然能正常出
点云（本 demo 实测 `/points` 7 Hz）。

**只有真的渲染不出来时**才需要兜底：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py
```

---

### C6. 实时因子只有 0.3~0.7

**症状**：Gazebo 界面右下角显示 `0.34 %`~`0.70 %`；`ros2 topic hz /points` 只有 3~7 Hz。

**先排除最大嫌疑：是不是跑了两个仿真？**

`pkill -f gazebo_lab1` **只杀 launch 脚本**，`gz sim` 本体的命令行是
`gz sim -r -v 3 .../lab1_room.sdf`，**匹配不到、会活下来**。于是重启 N 次就有 N 个仿真抢 GPU。

```bash
ps -ef | grep -E 'gz-sim|gz sim' | grep -v grep     # 必须只有一组
ros2 topic info /clock                              # Publisher count 必须是 1
```

正确清理：

```bash
pkill -f gz-sim; pkill -f 'gz sim'; pkill -f gz_sim
pkill -f rviz2; pkill -f sensor_prep; pkill -f patrol_driver; pkill -f parameter_bridge
sleep 3
ps -ef | grep -E 'gz-sim|rviz2' | grep -v grep      # 必须无输出
```

**两个仿真同时在跑还会导致** RViz 刷屏：

```
[tf2_buffer]: Detected jump back in time. Clearing TF buffer.
[rviz2]: Detected jump back in time. Resetting RViz.
```

因为两个 `/clock` 源的时间交替跳变。

**如果确认只有一个仿真还是慢**，按下面降负载（两处必须成对改）：

| 位置 | 改什么 |
|---|---|
| `models/lio_bot/model.sdf` 水平 `<samples>360</samples>` | → `180` |
| `config/params_gazebo.yaml` 的 `Horizon_SCAN: 360` | → `180` |
| `models/lio_bot/model.sdf` 垂直 `<samples>16</samples>` | → `8` |
| `config/params_gazebo.yaml` 的 `N_SCAN: 16` | → `8` |
| `gpu_lidar` 的 `<update_rate>10</update_rate>` | → `5` |

> LIO-SAM 用的是**消息里的仿真时间戳**算 dt，所以 RTF 0.7 对算法完全无影响 ——
> 200 Hz 的 IMU 在它眼里就是 200 Hz。

---

## D. launch 与参数

### D1. Demo 2 的 RViz 不启动

**症状**：`lio_sam_gazebo.launch.py` 起来了 4 个 LIO-SAM 节点，但进程列表里**根本没有
`rviz2-13` 那一行**（连 "process started" 都没有）。

**根因**：外层 launch 把 RViz 开关也命名成 `rviz`，同时又给 include 的
`gazebo_lab1.launch.py` 传了 `'rviz': 'false'`。**launch 的 include 参数会写进同一个
LaunchContext**，于是外层那个 `rviz` 在 `rviz2` 这个 action 被求值前就变成了 `false`，
`IfCondition` 直接跳过。

**修复**：外层开关改名成 `start_rviz`（和 include 传的 `rviz` 不撞名）

```python
rviz = LaunchConfiguration('start_rviz')
...
DeclareLaunchArgument('start_rviz', default_value='true')
```

**验证**

```bash
grep -n 'start_rviz' ros2_ws/src/lio_sam_gz_bringup/launch/lio_sam_gazebo.launch.py
grep -n 'start_rviz' ros2_ws/install/lio_sam_gz_bringup/share/lio_sam_gz_bringup/launch/lio_sam_gazebo.launch.py
# 两处都有 = 源码和 install 都生效了
```

> 改完之后的关闭方式是 `start_rviz:=false`（不再是 `rviz:=false`）。
> 临时救急也可以手动开：
> ```bash
> rviz2 -d $(ros2 pkg prefix lio_sam_gz_bringup)/share/lio_sam_gz_bringup/rviz/lio_sam_slam.rviz \
>       --ros-args -p use_sim_time:=true
> ```

---

### D2. RViz 里地图只有几段弧

**症状**：`map_global` 里点云一段一段呈圆弧状，看不到连续墙面。

**根因**：`params_gazebo.yaml` 沿用了 LIO-SAM ROS 1 版的**室外**默认值：

```yaml
globalMapVisualizationPoseDensity: 10.0   # 关键帧按 10 米体素抽稀
globalMapVisualizationLeafSize:   1.0     # 地图按 1 米体素抽稀
```

demo 的圆半径只有 1.25 m —— **10 米体素把圆周上所有关键帧压成 1 个**，
`map_global` 里就只剩一帧的点云（那条弧）。

**修复（室内尺度）**

```yaml
globalMapVisualizationSearchRadius: 50.0
globalMapVisualizationPoseDensity:  0.5
globalMapVisualizationLeafSize:     0.1
```

**注意**：这三个参数**只影响 RViz 里的可视化**。存 PCD 用的是 `mappingCornerLeafSize: 0.1` /
`mappingSurfLeafSize: 0.2`，所以 `save_map` 存出来的图一直是好的（比 RViz 里密十倍）。

---

### D3. LaunchConfigurationEquals 弃用警告

**症状**

```
UserWarning: The 'LaunchConfigurationEquals' and 'LaunchConfigurationNotEquals'
Conditions are deprecated. Use the 'EqualsSubstitution' ...
```

**结论**：无害。`launch` 的接口演进提示，demo 里用它做 `drive:=auto` 的条件判断。
想消掉可以改成 `IfCondition(EqualsSubstitution(LaunchConfiguration('drive'), 'auto'))`，
不改也不影响任何功能。

---

## E. 运行时诊断命令速查

```bash
# ---- 进程层（最容易出问题的地方）----
ps -ef | grep -E 'gz-sim|gz sim|rviz2|lio_sam' | grep -v grep
pkill -f gz-sim; pkill -f 'gz sim'; pkill -f rviz2

# ---- 话题：存在 vs 有数据 ----
ros2 topic list
ros2 topic info /clock            # Publisher count
ros2 topic info /points_raw
ros2 topic hz /points             # ≈ 7~10 Hz
ros2 topic hz /imu/data           # ≈ 140~200 Hz
ros2 topic echo /points --once --field fields     # 必须有 x,y,z,intensity,ring,time

# ---- gz 侧（绕过 ROS 直接看）----
gz topic -l | grep -iE 'points|imu'
timeout 3 gz topic -e -t /points_raw/points 2>&1 | head -c 300

# ---- TF ----
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link lidar_link

# ---- LIO-SAM 流水线（每一级都该 ≈7 Hz）----
ros2 topic hz /lio_sam/deskew/cloud_info
ros2 topic hz /lio_sam/feature/cloud_info
ros2 topic hz /lio_sam/mapping/odometry
ros2 topic hz /lio_sam/mapping/map_global     # ≈0.2 Hz，且必须先开 RViz（有订阅者才发布）
ros2 topic hz /lio_sam/odometry/imu           # ≈140 Hz

# ---- 资源 ----
free -h; watch -n 2 free -h
sudo dmesg -T | grep -iE 'out of memory|killed process' | tail
```

**按这个顺序二分**（定位"是仿真/桥接的锅还是 LIO-SAM 的锅"）：

```
① Gazebo 里车在动吗
② /points_raw 有 7~10 Hz 吗          → 桥接
③ /points 有 ring/time 字段吗         → sensor_prep
④ /imu/data 有 140~200 Hz 吗          → imu 系统插件
⑤ /lio_sam/deskew/cloud_info 有 7 Hz 吗 → imageProjection（N_SCAN/Horizon_SCAN 匹配）
⑥ /lio_sam/feature/cloud_info 有 7 Hz 吗 → featureExtraction（看 "Not enough features"）
⑦ /lio_sam/mapping/map_global 有点吗   → mapOptimization
```

---

## F. 从零到跑通的 checklist

- [ ] Ubuntu 24.04 + ROS 2 Jazzy（`ros-jazzy-desktop`）
- [ ] **加 8 GB swap**（8 GB 内存机器必做）
- [ ] `bash setup/check_env.sh`
- [ ] `bash setup/install_deps.sh`（含 `libmetis-dev` / `libsuitesparse-dev` / `fix_gtsam_cmake.sh`）
- [ ] `CMAKE_BUILD_PARALLEL_LEVEL=1` 编译，**不要删 build 目录**重链
- [ ] `ls ros2_ws/install/lio_sam/lib/lio_sam/` → 4 个可执行文件
- [ ] **`pkill -f gz-sim` 彻底清场**，确认 `ros2 topic info /clock` 只有 1 个发布者
- [ ] `ros2 topic hz /points`（7~10 Hz）**且** `ros2 topic hz /imu/data`（140~200 Hz）
- [ ] `ros2 launch lio_sam_gz_bringup lio_sam_gazebo.launch.py` → 4 个绿色横幅
- [ ] RViz：`Global Status: Ok` + 绿色圆轨迹 + `map_global` 有墙
- [ ] `ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.1, destination: /maps/lab1}"`

---

## G. 关键源码位置（想改代码时看这里）

| 文件 | 关键函数 | 看什么 |
|---|---|---|
| `src/imageProjection.cpp` | `cachePointCloud()` | 字段检查（`ring`/`time`）、`N_SCAN`/`Horizon_SCAN` 用在哪 |
| | `projectPointCloud()` | `rowIdn` 由 `ring` 得到、`columnIdn` 由水平角算 |
| | `deskewPoint()` | `time` 字段怎么用于去畸变 |
| `src/featureExtraction.cpp` | `calculateSmoothness()` / `extractFeatures()` | 一行扫描线上怎么挑角点/面点 |
| `src/imuPreintegration.cpp` | `IMUPreintegration::odometryHandler()` | 积分→加因子→优化→重播 |
| | `TransformFusion::imuOdometryHandler()` | **`lookupTransform(lidar, base_link)` 在这里**（外参 TF 必须存在） |
| `src/mapOptmization.cpp` | `laserCloudInfoHandler()` | 主循环入口 |
| | `extractSurroundingKeyFrames()` / `scan2MapOptimization()` | 局部子图 + LM |
| | `saveKeyFramesAndFactor()` | 什么时候加什么因子 |
| | `publishOdometry()` | **`odom→lidar_link` 的 child frame 是硬编码的 `"lidar_link"`** |
| `include/lio_sam/utility.hpp` | `ParamServer` | 所有参数名与默认值 |

**TF 谁发谁不发**（本 demo 的核心事实）：

| TF | 发布者 |
|---|---|
| `odom → base_link` | `TransformFusion`（`imuPreintegration.cpp`） |
| `odom → lidar_link` | `mapOptimization::publishOdometry()`（child frame 硬编码） |
| `map → odom` | **要自己发**（官方 `run.launch.py` 里也是 `static_transform_publisher`） |
| `base_link → lidar_link` / `base_link → imu_link` | **要自己发**（车体安装外参） |
