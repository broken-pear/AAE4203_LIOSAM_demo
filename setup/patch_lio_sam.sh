#!/usr/bin/env bash
# =============================================================================
# patch_lio_sam.sh —— 让官方 LIO-SAM 的 ros2 分支能在 Ubuntu 24.04 + GTSAM 4.2 上编过
#
# 只改 3 行，都是"该行在新环境里已经不适用"，不影响算法：
#
#   1) src/imuPreintegration.cpp:
#      #include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#      -> Ubuntu 的 libgtsam-dev（4.2.0+dfsg）不带 gtsam_unstable 模块，
#         而且 LIO-SAM 这个文件里根本没用 IncrementalFixedLagSmoother，注释掉即可。
#
#   2) src/imuPreintegration.cpp:
#      boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(...)
#      -> GTSAM 4.2 起智能指针从 boost::shared_ptr 换成了 std::shared_ptr，
#         改用 auto 接最省事，4.0 / 4.1 / 4.2 都能编。
#
#   3) CMakeLists.txt:
#      set(OpenCV_STATIC ON)
#      -> Ubuntu 的 apt 版 OpenCV 只有 .so，强制静态会找不到库，注释掉走动态链接。
#
# 幂等：可以重复执行。想撤销： git -C <LIO-SAM 目录> checkout .
#
# 用法: bash setup/patch_lio_sam.sh [LIO-SAM 目录]
# =============================================================================
set -euo pipefail

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${1:-$(cd "$HERE/.." && pwd)/ros2_ws/src/LIO-SAM}"

if [ ! -d "$SRC" ]; then
  echo -e "${RED}[error]${NC} 找不到 LIO-SAM 目录: $SRC"
  echo "       先跑: git clone https://github.com/TixiaoShan/LIO-SAM.git $SRC && git -C $SRC checkout ros2"
  exit 1
fi

IP="$SRC/src/imuPreintegration.cpp"
CML="$SRC/CMakeLists.txt"

[ -f "$IP" ]  || { echo -e "${RED}[error]${NC} 缺 $IP";  exit 1; }
[ -f "$CML" ] || { echo -e "${RED}[error]${NC} 缺 $CML"; exit 1; }

echo -e "${GREEN}==>${NC} 打补丁到 $SRC"

# ---- 1) gtsam_unstable include ------------------------------------------------
if grep -q '^#include <gtsam_unstable' "$IP"; then
  sed -i 's@^#include <gtsam_unstable.*$@// [patched] Ubuntu libgtsam-dev 不含 gtsam_unstable，且本文件未使用该头文件@' "$IP"
  echo "  [1/3] 已注释 gtsam_unstable include"
else
  echo "  [1/3] gtsam_unstable include 已处理过，跳过"
fi

# ---- 2) boost::shared_ptr -> auto --------------------------------------------
if grep -q 'boost::shared_ptr<gtsam::PreintegrationParams>' "$IP"; then
  # 首选：整句换成 auto（GTSAM 4.0~4.2 通吃）
  sed -i 's@boost::shared_ptr<gtsam::PreintegrationParams> p =@auto p =@' "$IP"
  # 兜底：万一行格式不一样（比如没有空格），退而求其次只换类型名
  if grep -q 'boost::shared_ptr<gtsam::PreintegrationParams>' "$IP"; then
    sed -i 's@boost::shared_ptr<gtsam::PreintegrationParams>@std::shared_ptr<gtsam::PreintegrationParams>@g' "$IP"
    echo "  [2/3] boost::shared_ptr -> std::shared_ptr（兜底方案）"
  else
    echo "  [2/3] boost::shared_ptr -> auto（适配 GTSAM >= 4.2）"
  fi
else
  echo "  [2/3] 没有 boost::shared_ptr 了，跳过"
fi

# ---- 3) OpenCV_STATIC --------------------------------------------------------
if grep -q 'set(OpenCV_STATIC ON)' "$CML"; then
  sed -i 's@set(OpenCV_STATIC ON)@# [patched] apt 版 OpenCV 只有动态库，不能强制静态@' "$CML"
  echo "  [3/3] 已注释 set(OpenCV_STATIC ON)"
else
  echo "  [3/3] OpenCV_STATIC 已处理过，跳过"
fi

echo
echo -e "${GREEN}==>${NC} 校验结果："
if grep -q '^#include <gtsam_unstable' "$IP"; then
  echo -e "  ${RED}✗${NC} gtsam_unstable 仍在"
else
  echo -e "  ${GREEN}✓${NC} gtsam_unstable include 已去掉"
fi
if grep -q 'boost::shared_ptr' "$IP"; then
  echo -e "  ${RED}✗${NC} boost::shared_ptr 仍在:"
  grep -n 'boost::shared_ptr' "$IP" || true
else
  echo -e "  ${GREEN}✓${NC} boost::shared_ptr 已去掉"
fi
grep -n 'MakeSharedU'  "$IP"  || true
grep -n 'OpenCV_STATIC' "$CML" || true

echo
echo -e "${YELLOW}提示${NC} 看改动细节： git -C \"$SRC\" diff"
