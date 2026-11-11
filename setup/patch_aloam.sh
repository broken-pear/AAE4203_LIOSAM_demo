#!/usr/bin/env bash
# =============================================================================
# patch_aloam.sh
#
# Makes the ROS 2 port of A-LOAM build on Ubuntu 24.04 + ROS 2 Jazzy.
#
# Upstream: https://github.com/G-Apple1/A-LOAM-ROS2
#           (ROS 2 Humble port of HKUST-Aerial-Robotics/A-LOAM)
#
# Everything below is an API/toolchain fix. NOT ONE LINE OF THE ALGORITHM IS
# TOUCHED, which matters for the report: the LOAM baseline in the comparison
# must be the published algorithm, not "whatever we could get to compile".
#
#   1) Ceres Solver 2.2 (the version in Ubuntu 24.04's libceres-dev) REMOVED
#      LocalParameterization in favour of Manifold:
#
#          ceres::LocalParameterization*               -> ceres::Manifold*
#          ceres::EigenQuaternionParameterization      -> ceres::EigenQuaternionManifold
#
#      Affected: laserOdometry.cpp (1 site), laserMapping.cpp (1 site).
#      With Ceres < 2.2 the sources are left untouched.
#
#   2) Ceres 2.2 headers require C++17 while CMakeLists.txt pins C++14.
#
#   3) laserMapping.cpp declares its two leaf-size parameters as
#      declare_parameter<float>. launch/YAML always delivers a double, and
#      rclcpp rejects that with InvalidParameterTypeException at startup:
#
#          parameter 'mapping_line_resolution' has invalid type:
#          expected [double] got [double] ... (float vs double)
#
#      Widening both to double lets the bringup set them from the launch file,
#      which is what keeps LOAM's map resolutions identical to LIO-SAM's.
#
#   4) CMakeLists.txt asks for rosbag2_cpp, which NONE of the sources include
#      or link. If the package is not installed, find_package(rosbag2_cpp)
#      aborts the whole build for nothing, so it is commented out only when it
#      is genuinely unavailable (present -> left untouched).
#
# Idempotent - safe to run any number of times.
# Undo: git -C <A-LOAM dir> checkout .
#
# Usage: bash setup/patch_aloam.sh [A-LOAM dir]
# =============================================================================
set -uo pipefail

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${1:-$(cd "$HERE/.." && pwd)/ros2_ws/src/A-LOAM-ROS2}"

LO="$SRC/src/laserOdometry.cpp"
LM="$SRC/src/laserMapping.cpp"
CML="$SRC/CMakeLists.txt"

if [ ! -d "$SRC" ]; then
  echo -e "${RED}[error]${NC} A-LOAM directory not found: $SRC"
  echo "        run: bash setup/install_deps.sh   (it clones the repo)"
  exit 1
fi
for f in "$LO" "$LM" "$CML"; do
  [ -f "$f" ] || { echo -e "${RED}[error]${NC} missing $f"; exit 1; }
done

echo -e "${GREEN}==>${NC} patching A-LOAM in $SRC"

# ---------------------------------------------------------------------------
# Detect the Ceres version. Two independent sources, because version.h is not
# installed by every packaging of Ceres and dpkg is missing on non-Debian.
# ---------------------------------------------------------------------------
ceres_major=""; ceres_minor=""
for hdr in /usr/include/ceres/version.h /usr/local/include/ceres/version.h; do
  if [ -f "$hdr" ]; then
    ceres_major="$(sed -n 's/^#define CERES_VERSION_MAJOR  *//p' "$hdr" | head -1)"
    ceres_minor="$(sed -n 's/^#define CERES_VERSION_MINOR  *//p' "$hdr" | head -1)"
    break
  fi
done
if [ -z "$ceres_major" ] && command -v dpkg-query >/dev/null 2>&1; then
  v="$(dpkg-query -W -f='${Version}' libceres-dev 2>/dev/null | sed 's/^[0-9]*://')"
  ceres_major="$(echo "$v" | cut -d. -f1)"
  ceres_minor="$(echo "$v" | cut -d. -f2)"
fi

needs_manifold=0
if [ -n "${ceres_major:-}" ] && [ -n "${ceres_minor:-}" ] \
   && [ "$ceres_major" -eq 2 ] 2>/dev/null && [ "$ceres_minor" -ge 2 ] 2>/dev/null; then
  needs_manifold=1
elif [ "${ceres_major:-0}" -gt 2 ] 2>/dev/null; then
  needs_manifold=1
fi
# Header-existence wins over version guessing.
if [ -f /usr/include/ceres/manifold.h ] && [ ! -f /usr/include/ceres/local_parameterization.h ]; then
  needs_manifold=1
fi

echo "  detected Ceres  : ${ceres_major:-?}.${ceres_minor:-?}"
echo "  needs Manifold  : $needs_manifold"

# --------------------------------------------------------------- 1) Manifold
if [ "$needs_manifold" = "1" ]; then
  for f in "$LO" "$LM"; do
    if grep -q 'ceres::EigenQuaternionParameterization' "$f"; then
      sed -i \
        -e 's@ceres::LocalParameterization@ceres::Manifold@g' \
        -e 's@ceres::EigenQuaternionParameterization@ceres::EigenQuaternionManifold@g' \
        "$f"
      echo "  [1/4] $(basename "$f"): LocalParameterization -> Manifold"
    else
      echo "  [1/4] $(basename "$f"): already patched (or not using the old API)"
    fi
  done
else
  echo "  [1/4] Ceres < 2.2 - LocalParameterization kept, nothing to do"
fi

# ------------------------------------------------------------- 2) C++17 pins
if grep -q 'set(CMAKE_CXX_STANDARD 14)' "$CML"; then
  sed -i 's@set(CMAKE_CXX_STANDARD 14)@set(CMAKE_CXX_STANDARD 17)@' "$CML"
  echo "  [2/4] CMakeLists.txt: C++14 -> C++17 (Ceres 2.2 headers need it)"
else
  echo "  [2/4] C++ standard already >= 17, skipping"
fi

# ------------------------------------------- 3) float params -> double params
changed=0
if grep -q 'declare_parameter<float>(' "$LM"; then
  sed -i 's@declare_parameter<float>(@declare_parameter<double>(@g' "$LM"
  changed=1
fi
if grep -q 'float lineRes' "$LM"; then
  sed -i 's@float lineRes@double lineRes@' "$LM"
  changed=1
fi
if grep -q 'float planeRes' "$LM"; then
  sed -i 's@float planeRes@double planeRes@' "$LM"
  changed=1
fi
if [ "$changed" = "1" ]; then
  echo "  [3/4] laserMapping.cpp: leaf-size params float -> double (YAML delivers double)"
else
  echo "  [3/4] leaf-size params already double, skipping"
fi

# ------------------------------------------------ 4) 可选依赖 rosbag2_cpp
# 源码里 grep 不到 rosbag2 的任何使用；它只是被写进了 CMakeLists。
# 没装这个包时 find_package(... REQUIRED) 会直接终止整个编译，所以按需摘掉。
have_rosbag2=0
if [ -n "${ROS_DISTRO:-}" ] && ros2 pkg prefix rosbag2_cpp >/dev/null 2>&1; then
  have_rosbag2=1
fi
if [ "$have_rosbag2" = "0" ]; then
  if grep -q '^find_package(rosbag2_cpp REQUIRED)' "$CML"; then
    sed -i 's@^find_package(rosbag2_cpp REQUIRED)@# [patched] 未安装 rosbag2_cpp；源码未使用该库@' "$CML"
    sed -i 's@^  rosbag2_cpp$@  # rosbag2_cpp@' "$CML"
    echo "  [4/4] rosbag2_cpp 不可用 -> 注释掉（源码未使用）"
  else
    echo "  [4/4] rosbag2_cpp 已处理过，跳过"
  fi
else
  echo "  [4/4] rosbag2_cpp 可用，保持原样"
fi

# -------------------------------------------------------------------- verify
echo
echo -e "${GREEN}==>${NC} verification"
rc=0
if [ "$needs_manifold" = "1" ]; then
  if grep -q 'ceres::LocalParameterization\|ceres::EigenQuaternionParameterization' "$LO" "$LM"; then
    echo -e "  ${RED}x${NC} old Ceres API still present:"
    grep -n 'LocalParameterization\|EigenQuaternionParameterization' "$LO" "$LM" || true
    rc=1
  else
    echo -e "  ${GREEN}ok${NC} no LocalParameterization / EigenQuaternionParameterization left"
  fi
fi
grep -q 'set(CMAKE_CXX_STANDARD 17)' "$CML" \
  && echo -e "  ${GREEN}ok${NC} CMAKE_CXX_STANDARD 17" \
  || { echo -e "  ${RED}x${NC} CMAKE_CXX_STANDARD is not 17"; rc=1; }
grep -q 'declare_parameter<double>("mapping_line_resolution"' "$LM" \
  && echo -e "  ${GREEN}ok${NC} mapping_*_resolution is double" \
  || echo -e "  ${YELLOW}!${NC} mapping_*_resolution not double (may still be fine)"

echo
echo -e "${YELLOW}tip${NC} full diff:    git -C \"$SRC\" diff"
echo -e "${YELLOW}tip${NC} revert:       git -C \"$SRC\" checkout ."
exit "$rc"
