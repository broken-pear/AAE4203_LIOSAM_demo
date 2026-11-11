#!/usr/bin/env bash
# =============================================================================
# fix_gtsam_cmake.sh
#
# Ubuntu 24.04 ships libgtsam-dev 4.2.0, but the package does NOT install the
# static helper archives that its own GTSAM-exports.cmake still references,
# e.g. /usr/lib/aarch64-linux-gnu/libCppUnitLite.a , so find_package(GTSAM)
# aborts with:
#
#   CMake Error at /usr/lib/cmake/GTSAM/GTSAM-exports.cmake:110 (message):
#     The imported target "CppUnitLite" references the file
#     ".../libCppUnitLite.a" but this file does not exist.
#
# Those archives belong to GTSAM's internal unit-test framework. LIO-SAM never
# includes or links them, so creating an EMPTY archive satisfies CMake's
# existence check and has no effect on the final binaries.
#
# IMPORTANT: GTSAM-exports.cmake writes paths as
#     ${_IMPORT_PREFIX}/<multiarch>/libXxx.a
# so grepping for a literal "/usr/lib/..." finds nothing. We have to resolve
# the real multiarch directory first, which is what this script does.
#
# Idempotent. Usage: bash setup/fix_gtsam_cmake.sh
# =============================================================================
set -uo pipefail

EXPORTS=""
for p in /usr/lib/cmake/GTSAM/GTSAM-exports.cmake \
         /usr/lib/*/cmake/GTSAM/GTSAM-exports.cmake; do
  if [ -f "$p" ]; then EXPORTS="$p"; break; fi
done

if [ -z "$EXPORTS" ]; then
  echo "[fix] GTSAM-exports.cmake not found - is libgtsam-dev installed?"
  exit 1
fi
echo "[fix] exports file : $EXPORTS"

MLIB="$(gcc -dumpmachine 2>/dev/null || dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null)"
if [ -z "$MLIB" ]; then
  echo "[fix] cannot determine multiarch triplet"
  exit 1
fi
echo "[fix] multiarch dir: /usr/lib/$MLIB"

if [ ! -e "/usr/lib/$MLIB/libgtsam.so" ]; then
  echo "  [warn] /usr/lib/$MLIB/libgtsam.so not found (LIO-SAM links this one)"
fi

names="$(grep -oE 'lib[A-Za-z0-9_+.-]*\.a' "$EXPORTS" | sort -u)"
if [ -z "$names" ]; then
  echo "[fix] no static archives referenced - nothing to do"
  exit 0
fi

rc=0
for n in $names; do
  f="/usr/lib/$MLIB/$n"
  if [ -e "$f" ]; then
    echo "  [ok  ] $f"
  else
    echo "  [make] $f   (empty archive, never linked by LIO-SAM)"
    if sudo ar rcs "$f"; then
      :
    else
      echo "  [fail] could not create $f"
      rc=1
    fi
  fi
done

if [ "$rc" -eq 0 ]; then
  echo "[fix] done. next: rm -rf ros2_ws/build/lio_sam && colcon build"
fi
exit "$rc"