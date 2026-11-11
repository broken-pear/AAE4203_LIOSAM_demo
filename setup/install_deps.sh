#!/usr/bin/env bash
# =============================================================================
# install_deps.sh —— 一次性把 LIO-SAM + Gazebo + 本 demo 装好并编译
#
# 用法：
#   source /opt/ros/jazzy/setup.bash
#   bash setup/install_deps.sh
#
# 五个步骤：
#   1. 检查 Ubuntu 24.04 + ROS 2 Jazzy + colcon
#   2. apt 装依赖（关键：libgtsam-dev 4.2，Ubuntu 24.04 官方源自带，
#      不用再 add PPA、不用编译 GTSAM —— 这是本 demo 省事的关键）
#   3. 克隆 LIO-SAM 官方 ros2 分支
#   4. 打 3 行补丁（见 patch_lio_sam.sh 里的说明）
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
    libgtsam-dev \
    libpcl-dev \
    libeigen3-dev \
    python3-numpy \
    python3-colcon-common-extensions

  # Gazebo：ROS 2 Jazzy 对应 Gazebo Harmonic，装 ros_gz 就行。
  # 不同 patch 版本 deb 名字略有差别，这里做两级尝试。
  if ! sudo apt install -y ros-jazzy-ros-gz; then
    warn "ros-jazzy-ros-gz 装不上，改成逐个装 ros_gz_sim / ros_gz_bridge"
    sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces
  fi
fi

# ------------------------------------------------------- 3. 拉 LIO-SAM 源码
info "3/5 准备 LIO-SAM 源码"
if [ -d "$SRC_DIR/LIO-SAM/.git" ]; then
  echo "  已存在，跳过 clone"
else
  git clone https://github.com/TixiaoShan/LIO-SAM.git "$SRC_DIR/LIO-SAM"
fi
git -C "$SRC_DIR/LIO-SAM" checkout ros2 2>/dev/null || warn "checkout ros2 分支失败（可能已经在 ros2 上了）"
echo "  当前分支: $(git -C "$SRC_DIR/LIO-SAM" rev-parse --abbrev-ref HEAD)"

# ------------------------------------------------------------------ 4. 补丁
info "4/5 打补丁"
bash "$HERE/patch_lio_sam.sh" "$SRC_DIR/LIO-SAM"

# ------------------------------------------------------- 4.5 修 libgtsam-dev
# Ubuntu 的 libgtsam-dev 少装了 CppUnitLite 等静态库，
# 会让 find_package(GTSAM) 直接报错，这里补上（LIO-SAM 不会真正链接它们）。
info "4.5/5 修补 GTSAM 的 CMake 导出文件"
bash "$HERE/fix_gtsam_cmake.sh" || warn "fix_gtsam_cmake.sh 返回非 0，继续尝试编译"
# ------------------------------------------------------------------ 5. 编译
info "5/5 colcon build（第一次大概 2~5 分钟；Jetson 上会久一些）"

# git 在 Windows 上无法保存可执行位，clone 下来 .py 是 644；
# 补一下，防止有人用 --symlink-install 时报 Permission denied。
chmod +x "$SRC_DIR"/lio_sam_gz_bringup/scripts/*.py 2>/dev/null || true
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

cat <<'EOF'

==============================================================
 全部完成！接下来：

   source ros2_ws/install/setup.bash
   ros2 launch lio_sam_gz_bringup lio_sam_gazebo.launch.py

 想先单独确认传感器和 Gazebo 正常，用 Demo 1：

   ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py
==============================================================
EOF
