#!/usr/bin/env bash
# =============================================================================
# fix_gtsam_cmake.sh
#
# Ubuntu 24.04 (noble) ships libgtsam-dev 4.2.0, but the package forgot to
# install the static helper archives that its own GTSAM-exports.cmake still
# references, e.g.
#
#   /usr/lib/aarch64-linux-gnu/libCppUnitLite.a
#
# so find_package(GTSAM) aborts with:
#
#   CMake Error at /usr/lib/cmake/GTSAM/GTSAM-exports.cmake:110 (message):
#     The imported target "CppUnitLite" references the file
#     ".../libCppUnitLite.a" but this file does not exist.
#
# CppUnitLite is GTSAM's internal unit-test framework. LIO-SAM never includes
# it and never links it, so an EMPTY static archive is enough to satisfy
# CMake's existence check.
#
# Safe and idempotent: only creates files that are actually missing.
# Usage: bash setup/fix_gtsam_cmake.sh
# =============================================================================
set -uo pipefail

EXPORTS=/usr/lib/cmake/GTSAM/GTSAM-exports.cmake
if [ -f /usr/lib/x86_64-linux-gnu/cmake/GTSAM/GTSAM-exports.cmake ]; then
  EXPORTS=/usr/lib/x86_64-linux-gnu/cmake/GTSAM/GTSAM-exports.cmake
elif [ -f /usr/lib/aarch64-linux-gnu/cmake/GTSAM/GTSAM-exports.cmake ]; then
  EXPORTS=/usr/lib/aarch64-linux-gnu/cmake/GTSAM/GTSAM-exports.cmake
fi

if [ ! -f "$EXPORTS" ]; then
  echo "[fix] GTSAM-exports.cmake not found - is libgtsam-dev installed?"
  exit 1
fi

echo "[fix] scanning $EXPORTS"

# ---- static archives: create empty ones if missing -------------------------
for f in $(grep -oE '/usr/lib[^"]*\.a' "$EXPORTS" | sort -u); do
  if [ -e "$f" ]; then
    echo "  [ok  ] $f"
  else
    echo "  [make] $f   (empty archive, never linked)"
    sudo mkdir -p "$(dirname "$f")"
    sudo ar rcs "$f"
  fi
done

# ---- shared objects: report only (cannot be faked) ------------------------
rc=0
for f in $(grep -oE '/usr/lib[^"]*\.so[^"]*' "$EXPORTS" | sort -u); do
  if [ -e "$f" ]; then
    echo "  [ok  ] $f"
  else
    echo "  [MISS] $f   <-- real missing library, cannot be faked"
    rc=1
  fi
done

if [ "$rc" -eq 0 ]; then
  echo "[fix] done. re-run: colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
fi
exit "$rc"