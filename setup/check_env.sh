#!/usr/bin/env bash
# =============================================================================
# check_env.sh —— 开跑前先体检，缺什么会直接告诉你
#
# 用法：
#   source /opt/ros/jazzy/setup.bash
#   bash setup/check_env.sh
# =============================================================================
set -uo pipefail

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}[ OK ]${NC} $*"; }
bad()  { echo -e "  ${RED}[FAIL]${NC} $*"; FAILED=1; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }

FAILED=0
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$HERE/.." && pwd)/ros2_ws"

echo "=============================================================="
echo " 1. 操作系统 / ROS 版本"
echo "=============================================================="
if [ -f /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  echo "  OS: $PRETTY_NAME"
  if [ "${VERSION_ID:-}" = "24.04" ]; then
    ok "Ubuntu 24.04（和 ROS 2 Jazzy / Gazebo Harmonic 匹配）"
  else
    warn "ROS 2 Jazzy 官方只支持 Ubuntu 24.04，当前是 ${VERSION_ID:-未知}"
  fi
else
  bad "找不到 /etc/os-release，不是 Linux？"
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  bad "ROS 环境没 source。请先执行: source /opt/ros/jazzy/setup.bash"
else
  if [ "$ROS_DISTRO" = "jazzy" ]; then
    ok "ROS_DISTRO = $ROS_DISTRO"
  else
    bad "ROS_DISTRO = $ROS_DISTRO，本 demo 针对 jazzy"
  fi
fi

echo
echo "=============================================================="
echo " 2. 关键的 apt 包"
echo "=============================================================="
check_pkg() {   # $1 = 包名, $2 = 说明
  if dpkg -s "$1" >/dev/null 2>&1; then
    ok "$1  ($2)"
  else
    bad "$1  ($2) —— sudo apt install $1"
  fi
}
check_pkg ros-jazzy-ros-gz              "Gazebo Harmonic + ros_gz_bridge"
check_pkg ros-jazzy-perception-pcl      "pcl_conversions / pcl_msgs"
check_pkg ros-jazzy-vision-opencv       "cv_bridge / OpenCV"
check_pkg ros-jazzy-tf2-sensor-msgs     "LIO-SAM 需要"
check_pkg ros-jazzy-xacro               "LIO-SAM 官方 launch 需要"
check_pkg libgtsam-dev                  "GTSAM 4.2（Ubuntu 24.04 自带，不用编译）"
check_pkg libceres-dev                  "Ceres 2.2（A-LOAM 的优化器）"
check_pkg libgoogle-glog-dev            "glog（A-LOAM 链接）"
check_pkg libpcl-dev                    "PCL"
check_pkg python3-numpy                 "sensor_prep.py 需要"

echo
echo "=============================================================="
echo " 3. GTSAM 的 CMake 配置（find_package(GTSAM) 能不能找到）"
echo "=============================================================="
if [ -f /usr/lib/x86_64-linux-gnu/cmake/GTSAM/GTSAMConfig.cmake ] || \
   [ -f /usr/lib/cmake/GTSAM/GTSAMConfig.cmake ]; then
  ok "GTSAMConfig.cmake 存在"
else
  bad "找不到 GTSAMConfig.cmake，LIO-SAM 编译时会 find_package(GTSAM) 失败"
fi

echo
echo "=============================================================="
echo " 4. 工作空间"
echo "=============================================================="
if [ -d "$WS_DIR/src/LIO-SAM" ]; then
  ok "LIO-SAM 源码在 $WS_DIR/src/LIO-SAM"
  if grep -q '^// \[patched' "$WS_DIR/src/LIO-SAM/src/imuPreintegration.cpp" 2>/dev/null; then
    ok "imuPreintegration.cpp 已打过补丁"
  else
    warn "还没打补丁 —— 运行 bash setup/install_deps.sh 会自动打"
  fi
else
  bad "没有 $WS_DIR/src/LIO-SAM —— 运行 bash setup/install_deps.sh"
fi

if [ -d "$WS_DIR/src/A-LOAM-ROS2" ]; then
  ok "A-LOAM 源码在 $WS_DIR/src/A-LOAM-ROS2 (commit $(git -C "$WS_DIR/src/A-LOAM-ROS2" rev-parse --short HEAD 2>/dev/null || echo '?'))"
  if grep -rqs 'ceres::Manifold\|ceres::LocalParameterization' \
        "$WS_DIR/src/A-LOAM-ROS2/src/laserOdometry.cpp"; then
    ok "laserOdometry.cpp 已包含求解器参数化（Manifold 或旧 API）"
  else
    bad "laserOdometry.cpp 里找不到 ceres 参数化 —— 源码可能被改坏了"
  fi
  if [ -f /usr/include/ceres/manifold.h ] && [ ! -f /usr/include/ceres/local_parameterization.h ]; then
    if grep -qs 'ceres::LocalParameterization\|ceres::EigenQuaternionParameterization' \
         "$WS_DIR/src/A-LOAM-ROS2/src/laserOdometry.cpp" \
         "$WS_DIR/src/A-LOAM-ROS2/src/laserMapping.cpp"; then
      bad "Ceres 2.2 但没有打 Manifold 补丁 —— 运行 bash setup/patch_aloam.sh"
    else
      ok "Ceres 2.2 + 已打 Manifold 补丁"
    fi
  fi
else
  bad "没有 $WS_DIR/src/A-LOAM-ROS2 —— 运行 bash setup/install_deps.sh"
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
  ok "工作空间已编译（$WS_DIR/install）"
else
  warn "还没编译（$WS_DIR/install 不存在）"
fi

# 两套算法的可执行文件
for exe in "$WS_DIR/install/lio_sam/lib/lio_sam/lio_sam_mapOptimization" \
           "$WS_DIR/install/aloam_velodyne_ros2/lib/aloam_velodyne_ros2/alaserMapping"; do
  if [ -x "$exe" ]; then
    ok "可执行: ${exe#$WS_DIR/}"
  elif [ -d "$WS_DIR/install" ]; then
    warn "缺少 ${exe#$WS_DIR/}（编译失败或还没编）"
  fi
done

echo
echo "=============================================================="
echo " 5. 图形 / GPU（Gazebo 的 gpu_lidar 需要渲染）"
echo "=============================================================="
if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
  ok "有图形界面（DISPLAY=${DISPLAY:-}）"
else
  warn "没有 DISPLAY。WSL2/远程的话需要 WSLg 或者 X11 转发，否则 Gazebo 开不出来"
fi
if [ -e /dev/dri ] || command -v nvidia-smi >/dev/null 2>&1; then
  ok "检测到 GPU 设备"
else
  warn "没检测到 /dev/dri 或 nvidia-smi，可能要走软件渲染："
  warn "  export LIBGL_ALWAYS_SOFTWARE=1"
fi

echo
echo "=============================================================="
if [ "$FAILED" -eq 0 ]; then
  echo -e " ${GREEN}体检通过，可以开跑。${NC}"
else
  echo -e " ${RED}有项目没通过，先把上面标 [FAIL] 的装好。${NC}"
fi
echo "=============================================================="
exit "$FAILED"
