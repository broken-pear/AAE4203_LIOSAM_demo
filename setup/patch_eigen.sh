#!/usr/bin/env bash
# =============================================================================
# patch_eigen.sh
#
# LIO-SAM's CMakeLists.txt calls
#     find_package(Eigen REQUIRED)
# but Eigen3 only installs Eigen3Config.cmake - there is no EigenConfig.cmake
# and CMake does not ship a FindEigen.cmake module, so CMake aborts with:
#
#   By not providing "FindEigen.cmake" in CMAKE_MODULE_PATH this project has
#   asked CMake to find a package configuration file provided by "Eigen"
#
# Nothing in the build actually consumes Eigen's CMake variables (the headers
# arrive through PCL / GTSAM), so switching the call to Eigen3 is enough.
#
# Idempotent. Usage: bash setup/patch_eigen.sh [LIO-SAM dir]
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${1:-$(cd "$HERE/.." && pwd)/ros2_ws/src/LIO-SAM}"
CML="$SRC/CMakeLists.txt"

if [ ! -f "$CML" ]; then
  echo "[eigen] not found: $CML"
  exit 1
fi

echo "[eigen] patching $CML"
if grep -q '^find_package(Eigen REQUIRED)' "$CML"; then
  sed -i 's@^find_package(Eigen REQUIRED)@find_package(Eigen3 REQUIRED)@' "$CML"
  echo "  [4/4] find_package(Eigen) -> find_package(Eigen3)"
else
  echo "  find_package(Eigen ...) not found or already patched - skipping"
fi

# ament_target_dependencies() validates every name against packages that were
# actually find_package()d, so the target listing "GTSAM Eigen" would now fail
# with: ament_target_dependencies() the passed package name 'Eigen' was not found
if grep -q 'GTSAM Eigen)' "$CML"; then
  sed -i 's@GTSAM Eigen)@GTSAM Eigen3)@' "$CML"
  echo "  [5/5] ament_target_dependencies: GTSAM Eigen -> GTSAM Eigen3"
else
  echo "  ament_target_dependencies GTSAM Eigen - already patched"
fi

echo "[eigen] now:"
grep -n 'find_package(Eigen' "$CML" || true