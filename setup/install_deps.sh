#!/usr/bin/env bash
# =============================================================================
# install_deps.sh —— 一次性把 两条管线 + Gazebo + 本 demo 装好并编译
#
# 装的是两套算法，用来做公平对比：
#   * LIO-SAM   官方 ros2 分支          —— 比较组的 SOTA
#   * A-LOAM    A-LOAM 的 ROS 2 移植    —— baseline（纯激光 LOAM）
#
# 用法：
#   source /opt/ros/jazzy/setup.bash
#   bash setup/install_deps.sh
#
# 五个步骤：
#   1. 检查 Ubuntu 24.04 + ROS 2 Jazzy + colcon
#   2. apt 装依赖（关键：libgtsam-dev 4.2、libceres-dev 2.2，Ubuntu 24.04 官方源
#      自带，不用 add PPA、不用自己编 GTSAM/Ceres —— 这是本 demo 省事的关键）
#   3. 克隆 LIO-SAM（ros2 分支）+ A-LOAM-ROS2（pin 到 commit）
#   4. 打补丁（LIO-SAM 5 处、A-LOAM 3 处，都是 API 版本差异，不碰算法）
#   5. colcon build
#
# 环境变量：
#   SKIP_APT=1   跳过 apt（已经装好了、或者没有 sudo 权限时用）
# =============================================================================
set -euo pipefail

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
info() { echo -e "\n${GREEN}==>${NC} $*"; }
warn() { echo -e "${YELLOW}[warn]${NC} $*"; }
die()  { echo -e "${RED}[error]${NC} $*"; exit 1; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
WS_DIR="$ROOT/ros2_ws"
SRC_DIR="$WS_DIR/src"

# A-LOAM 的 pinned commit（和 demo.repos 里的必须一致）
ALOAM_SHA="${ALOAM_SHA:-0674b11b081058abc7c6ca74180cc07f58700bd8}"

# ---------------------------------------------------------------- 1. 环境检查
info "1/5 检查环境"

[ -f /etc/os-release ] || die "这不是 Linux"
. /etc/os-release
if [ "${VERSION_ID:-}" != "24.04" ]; then
  warn "当前 Ubuntu ${VERSION_ID:-?}，本 demo 针对 24.04 + Jazzy，可能踩坑"
fi

if [ -z "${ROS_DISTRO:-}" ] || [ "$ROS_DISTRO" != "jazzy" ]; then
  die "请先 source /opt/ros/jazzy/setup.bash（当前 ROS_DISTRO=${ROS_DISTRO:-空}）"
fi
command -v colcon >/dev/null 2>&1 || die "没装 colcon: sudo apt install python3-colcon-common-extensions"
command -v git    >/dev/null 2>&1 || die "没装 git: sudo apt install git"

mkdir -p "$SRC_DIR"

# ------------------------------------------------------------------- 2. apt
if [ "${SKIP_APT:-0}" = "1" ]; then
  warn "SKIP_APT=1，跳过 apt 安装"
else
  info "2/5 apt 安装依赖（需要 sudo 密码）"
  sudo apt update

  sudo apt install -y \
    build-essential cmake git \
    ros-jazzy-perception-pcl \
    ros-jazzy-pcl-msgs \
    ros-jazzy-vision-opencv \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-eigen \
    ros-jazzy-xacro \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    libgtsam-dev \
    libpcl-dev \
    libeigen3-dev \
    python3-numpy \
    python3-yaml \
    python3-matplotlib \
    python3-colcon-common-extensions

  # A-LOAM 的 CMakeLists 里还写了 rosbag2_cpp，但源码一行都没用到它。
  # 装得上最好；装不上也不影响 —— patch_aloam.sh 会在编译前按需摘掉这一项。
  sudo apt install -y ros-jazzy-rosbag2-cpp \
    || warn "ros-jazzy-rosbag2-cpp 装不上（无所谓，patch_aloam.sh 会注释掉它）"

  # A-LOAM 的依赖：Ceres 做优化、glog 做日志。
  # Ubuntu 24.04 的 libceres-dev 是 2.2，删了 LocalParameterization —— 见 patch_aloam.sh
  sudo apt install -y libceres-dev libgoogle-glog-dev || \
    die "libceres-dev / libgoogle-glog-dev 装不上，A-LOAM 编不了"

  # Gazebo：ROS 2 Jazzy 对应 Gazebo Harmonic，装 ros_gz 就行。
  # 不同 patch 版本 deb 名字略有差别，这里做两级尝试。
  if ! sudo apt install -y ros-jazzy-ros-gz; then
    warn "ros-jazzy-ros-gz 装不上，改成逐个装 ros_gz_sim / ros_gz_bridge"
    sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces
  fi
fi

# --------------------------------------------------------- 3. 拉两套算法源码
info "3/5 准备源码（LIO-SAM + A-LOAM）"

# --- LIO-SAM ---------------------------------------------------------------
if [ -d "$SRC_DIR/LIO-SAM/.git" ]; then
  echo "  LIO-SAM: 已存在，跳过 clone"
else
  git clone https://github.com/TixiaoShan/LIO-SAM.git "$SRC_DIR/LIO-SAM"
fi
git -C "$SRC_DIR/LIO-SAM" checkout ros2 2>/dev/null || warn "checkout ros2 分支失败（可能已经在 ros2 上了）"
echo "  LIO-SAM 当前分支: $(git -C "$SRC_DIR/LIO-SAM" rev-parse --abbrev-ref HEAD)"

# --- A-LOAM ---------------------------------------------------------------
# 目录名刻意用 A-LOAM-ROS2（和上游仓库同名），里面的 package 名是 aloam_velodyne_ros2
ALOAM_DIR="$SRC_DIR/A-LOAM-ROS2"
if [ -d "$ALOAM_DIR/.git" ]; then
  echo "  A-LOAM : 已存在，跳过 clone"
else
  git clone https://github.com/G-Apple1/A-LOAM-ROS2.git "$ALOAM_DIR"
fi
# pin 到固定 commit：报告要能复现"同一个 LOAM"
if git -C "$ALOAM_DIR" cat-file -e "${ALOAM_SHA}^{commit}" 2>/dev/null; then
  git -C "$ALOAM_DIR" checkout -q "$ALOAM_SHA"
  echo "  A-LOAM 当前 commit: $(git -C "$ALOAM_DIR" rev-parse --short HEAD)  (pinned)"
else
  git -C "$ALOAM_DIR" fetch --all --quiet || true
  if git -C "$ALOAM_DIR" cat-file -e "${ALOAM_SHA}^{commit}" 2>/dev/null; then
    git -C "$ALOAM_DIR" checkout -q "$ALOAM_SHA"
    echo "  A-LOAM 当前 commit: $(git -C "$ALOAM_DIR" rev-parse --short HEAD)  (pinned)"
  else
    warn "找不到 commit $ALOAM_SHA，停在默认分支 $(git -C "$ALOAM_DIR" rev-parse --short HEAD)"
    warn "  报告里请记录实际 commit；或改 ALOAM_SHA=<sha> 重跑本脚本"
  fi
fi

# ------------------------------------------------------------------ 4. 补丁
info "4/5 打补丁"

# LIO-SAM：gtsam_unstable / boost::shared_ptr / Eigen / OpenCV_STATIC
bash "$HERE/patch_lio_sam.sh" "$SRC_DIR/LIO-SAM"

# A-LOAM：Ceres 2.2 Manifold / C++17 / float->double 参数
bash "$HERE/patch_aloam.sh" "$ALOAM_DIR"

# ------------------------------------------------------- 4.5 修 libgtsam-dev
# Ubuntu 的 libgtsam-dev 少装了 CppUnitLite 等静态库，
# 会让 find_package(GTSAM) 直接报错，这里补上（LIO-SAM 不会真正链接它们）。
info "4.5/5 修补 GTSAM 的 CMake 导出文件"
bash "$HERE/fix_gtsam_cmake.sh" || warn "fix_gtsam_cmake.sh 返回非 0，继续尝试编译"

# ------------------------------------------------------------------ 5. 编译
info "5/5 colcon build（第一次大概 3~8 分钟；Jetson 上会久一些）"

# git 在 Windows 上无法保存可执行位，clone 下来 .py 是 644；
# 补一下，防止有人用 --symlink-install 时报 Permission denied。
chmod +x "$SRC_DIR"/lio_sam_gz_bringup/scripts/*.py 2>/dev/null || true
chmod +x "$SRC_DIR"/lio_sam_bench/scripts/*.py 2>/dev/null || true
chmod +x "$HERE"/*.sh 2>/dev/null || true

# 内存紧张的机器（Jetson Orin Nano 8G 等）可以这样限并发：
#   export CMAKE_BUILD_PARALLEL_LEVEL=2
if [ -n "${CMAKE_BUILD_PARALLEL_LEVEL:-}" ]; then
  echo "  CMAKE_BUILD_PARALLEL_LEVEL=$CMAKE_BUILD_PARALLEL_LEVEL（已限制编译并发）"
fi

cd "$WS_DIR"
# 注意：这里不用 --symlink-install。
# 因为 install(PROGRAMS) 在 symlink 模式下只是建了个软链接，
# 而 Windows/网盘拷过来的 .py 往往没有可执行位，ros2 run 会报权限错误。
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ------------------------------------------------------------------ 收尾提示
cat <<'EOF'

==============================================================
 全部完成！两套算法都编好了：

   ros2_ws/install/lio_sam/lib/lio_sam/             (4 个可执行)
   ros2_ws/install/aloam_velodyne_ros2/lib/aloam_velodyne_ros2/  (3 个可执行)

 先单独确认传感器 / Gazebo 正常，用 Demo 1：

   source ros2_ws/install/setup.bash
   ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py

 只跑 LIO-SAM（Demo 2）：

   ros2 launch lio_sam_gz_bringup lio_sam_gazebo.launch.py

 只跑 LOAM baseline（Demo 3）：

   ros2 launch lio_sam_gz_bringup loam_gazebo.launch.py

 两者同屏对比 + 录真值（Demo 4）：

   ros2 launch lio_sam_bench compare_gazebo.launch.py

 正式跑对比实验（录包 -> 回放 -> 出 CSV/表）：

   bash experiments/run_bench.sh --help
==============================================================
EOF
