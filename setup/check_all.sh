#!/usr/bin/env bash
# =============================================================================
# check_all.sh —— 一条命令跑完所有"不需要 Gazebo 也能做"的检查
#
#   bash setup/check_all.sh            # 全部跑，最后一张汇总表
#   bash setup/check_all.sh -v         # 同时把每个检查的输出打出来
#
# 七个检查覆盖：
#   _selfcheck.py      静态一致性（YAML/SDF/launch/参数对齐/场景可行性/bash -n/编码）
#   _test_pack.py      点云打包与 ring/time 计算
#   _test_metrics.py   评测器的解析算例（2% 尺度 → 恰好 25 mm）
#   _test_pipeline.py  CSV → 指标 → 图 → PPT 全链路（没有 python-pptx 时自动跳过 PPT 那段）
#   _test_nodes.py     四个 rclpy 节点的逻辑（用桩真跑）
#   _test_sanity.py    体检规则本身（真值过短要判 FAIL、尺度异常要 WARN）
#   _test_run_bench.py run_bench.sh 的完整编排（假 ros2 + 工作区副本）
#
# 退出码 0 = 全绿。任何一项失败都会在最后标出来。
# =============================================================================
set -uo pipefail

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
PY="${PYTHON:-python3}"
VERBOSE=0
[ "${1:-}" = "-v" ] && VERBOSE=1

command -v "$PY" >/dev/null 2>&1 || { echo -e "${RED}[error]${NC} 找不到 $PY"; exit 1; }
[ -d "$ROOT/.git" ] || echo -e "${YELLOW}[warn]${NC} $ROOT 不是 git 仓库（部分检查会跳过需要 git 的项）"

# ------------------------------------------------------------------ 解释器预检
# 缺 PyYAML 时后面五六项都会以"奇怪的方式"失败，报错却指向业务逻辑。
# 与其让人去猜，不如在这里一次说清该装什么。
DEPS_OUT="$("$PY" - <<'PYEOF' 2>&1
import importlib.util
need = {'yaml': 'python3-yaml', 'numpy': 'python3-numpy'}
opt = {'matplotlib': 'python3-matplotlib', 'pptx': 'python3-pptx'}
miss = [pkg for mod, pkg in need.items() if importlib.util.find_spec(mod) is None]
opt_miss = [pkg for mod, pkg in opt.items() if importlib.util.find_spec(mod) is None]
print('MISSING=' + ','.join(miss))
print('OPTIONAL_MISSING=' + ','.join(opt_miss))
PYEOF
)" || { echo -e "${RED}[error]${NC} $PY 跑不起来"; exit 1; }

MISSING="$(printf '%s\n' "$DEPS_OUT" | sed -n 's/^MISSING=//p')"
OPT_MISSING="$(printf '%s\n' "$DEPS_OUT" | sed -n 's/^OPTIONAL_MISSING=//p')"
if [ -n "$MISSING" ]; then
  echo -e "${RED}[error]${NC} $PY 缺少必需依赖：${MISSING//,/ }"
  echo "        装一下： sudo apt install -y ${MISSING//,/ }"
  echo "        （或者用一个装好这些库的解释器：PYTHON=/usr/bin/python3 bash setup/check_all.sh）"
  exit 2
fi
if [ -n "$OPT_MISSING" ]; then
  echo -e "${YELLOW}[warn]${NC} 可选依赖没装：${OPT_MISSING//,/ } —— 相关检查会自动跳过，不算失败"
fi

CHECKS=(
  "_selfcheck.py|静态一致性"
  "_test_pack.py|点云打包"
  "_test_metrics.py|评测器算例"
  "_test_pipeline.py|报告链"
  "_test_nodes.py|节点逻辑"
  "_test_sanity.py|体检规则"
  "_test_run_bench.py|实验编排"
)

NAMES=(); CODES=(); SECS=()
echo "=============================================================="
echo " 离线检查（不需要 Gazebo / 不需要算法在跑）"
echo " python: $("$PY" -c 'import sys; print(sys.version.split()[0])' 2>/dev/null || echo '?')"
echo " 仓库  : $ROOT"
echo "=============================================================="

for entry in "${CHECKS[@]}"; do
  script="${entry%%|*}"; label="${entry##*|}"
  path="$HERE/$script"
  if [ ! -f "$path" ]; then
    printf '  %-22s %s\n' "$label" "${RED}缺失 $script${NC}"
    NAMES+=("$label"); CODES+=("2"); SECS+=("0")
    continue
  fi
  printf '  %-22s ... ' "$label"
  t0=$(date +%s)
  out="$("$PY" "$path" 2>&1)"; rc=$?
  t1=$(date +%s)
  if [ "$rc" -eq 0 ]; then
    printf '%s  (%ss)\n' "${GREEN}PASS${NC}" "$((t1 - t0))"
  else
    printf '%s  (%ss, exit=%s)\n' "${RED}FAIL${NC}" "$((t1 - t0))" "$rc"
  fi
  if [ "$VERBOSE" -eq 1 ] || [ "$rc" -ne 0 ]; then
    printf '%s\n' "$out" | sed 's/^/      /'
  fi
  # 只留最后一行结论，方便回看
  NAMES+=("$label"); CODES+=("$rc"); SECS+=("$((t1 - t0))")
done

echo
echo "=============================================================="
echo " 汇总"
echo "=============================================================="
fails=0
for i in "${!NAMES[@]}"; do
  if [ "${CODES[$i]}" -eq 0 ]; then
    printf '  %-22s %s %ss\n' "${NAMES[$i]}" "${GREEN}OK  ${NC}" "${SECS[$i]}"
  else
    printf '  %-22s %s exit=%s\n' "${NAMES[$i]}" "${RED}FAIL${NC}" "${CODES[$i]}"
    fails=$((fails + 1))
  fi
done
echo
if [ "$fails" -eq 0 ]; then
  echo -e " ${GREEN}全部通过${NC}（${#NAMES[@]} 项检查）"
  echo " 这一步过了，剩下的问题只可能出在真机链路（Gazebo / 传感器 / 算法本身）。"
  exit 0
fi
echo -e " ${RED}$fails 项失败${NC} —— 先修这些，再跑 experiments/run_bench.sh"
exit 1
