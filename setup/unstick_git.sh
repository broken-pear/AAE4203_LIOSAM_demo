#!/usr/bin/env bash
# =============================================================================
# unstick_git.sh —— git pull 被"本地改动"挡住时，先判断清楚再动
#
# 典型场景（这套 demo 踩过三次）：
#   $ git pull
#   error: Your local changes to the following files would be overwritten by merge:
#           ros2_ws/src/lio_sam_bench/scripts/gt_sync.py
#           ...
#   根因往往不是"你改过文件"，而是 install_deps.sh 的 chmod +x 让**文件模式**
#   变了（内容一模一样），git 把这当成未提交改动。
#
# 这个脚本做三件事，**先诊断、再决定**：
#   1. 列出所有被改动文件，并把它们分成两类：
#        - 只有 mode 变化（内容没变）→ 安全，可以忽略/丢弃
#        - 有内容变化                → 你自己改的，绝不会动它
#   2. 如果全都是 mode 变化：把 core.fileMode 设成 false（一次设置，永久生效），
#      这样 git 以后不再把可执行位当改动 —— 冲突从此不再出现。
#      这一步**不修改任何文件内容**。
#   3. 如果有内容变化：列出它们并停下，让你自己决定 stash 还是提交。
#
# 用法：
#   bash setup/unstick_git.sh            # 诊断 + 设置 core.fileMode false（安全）
#   bash setup/unstick_git.sh --dry-run  # 只诊断，什么都不改
#   bash setup/unstick_git.sh --discard  # 诊断 + 设置 + 把纯 mode 改动 checkout 掉
#                                        #   （内容改动仍然不动）
# =============================================================================
set -uo pipefail

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
info() { echo -e "${GREEN}==>${NC} $*"; }
warn() { echo -e "${YELLOW}[warn]${NC} $*"; }
die()  { echo -e "${RED}[error]${NC} $*"; exit 1; }

DRY=0; DISCARD=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    --discard) DISCARD=1 ;;
    -h|--help) sed -n '2,32p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "不认识的参数: $a" ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT" || die "进不去 $ROOT"

git rev-parse --is-inside-work-tree >/dev/null 2>&1 || die "$ROOT 不是 git 仓库"

info "当前 HEAD: $(git rev-parse --short HEAD) $(git log -1 --format=%s | cut -c1-50)"
echo

# ---------------------------------------------------------------- 1. 分类
# 用 porcelain 拿到状态，再逐个文件判断"到底是 mode 还是内容"
MODIFIED=$(git status --porcelain | awk '$1 ~ /M|A/ {print $2}')
if [ -z "$MODIFIED" ]; then
  info "工作区是干净的 —— 没有东西会挡住 git pull"
  echo
  info "如果 pull 仍然失败，那是网络/远端的问题，不是本地改动"
  exit 0
fi

MODE_ONLY=(); CONTENT=()
for f in $MODIFIED; do
  # 纯 mode 变化在 --numstat 里是 "0 0 <file>"（增删行数都是 0）；
  # 有内容改动才是非零（二进制文件显示 "-"）。
  # 用 grep 找数字是错的 —— 那个 "0" 也会被匹配到。
  line="$(git diff --numstat -- "$f" | head -1)"
  add="$(printf '%s\n' "$line" | awk '{print $1}')"
  del="$(printf '%s\n' "$line" | awk '{print $2}')"
  if [ -z "$line" ] || { [ "${add:-0}" = "0" ] && [ "${del:-0}" = "0" ]; }; then
    MODE_ONLY+=("$f")
  else
    CONTENT+=("$f")
  fi
done

echo "被改动的文件：$(echo "$MODIFIED" | wc -w) 个"
if [ ${#MODE_ONLY[@]} -gt 0 ]; then
  echo
  echo -e "${GREEN}只有文件模式变化（内容未变，安全）：${NC}"
  for f in "${MODE_ONLY[@]}"; do
    printf '  [mode] %s\n' "$f"
  done
fi
if [ ${#CONTENT[@]} -gt 0 ]; then
  echo
  echo -e "${RED}有内容改动（脚本不会碰这些）：${NC}"
  for f in "${CONTENT[@]}"; do
    printf '  %s\n' "$f"
    git diff --numstat -- "$f" | sed 's/^/    /'
  done
fi

# ---------------------------------------------------------------- 2. 行动
if [ ${#CONTENT[@]} -gt 0 ]; then
  echo
  warn "你有 ${#CONTENT[@]} 个文件是真的改过内容。请自己决定："
  warn "  想保留  → git stash push -m 'my work' ，pull 完再 git stash pop"
  warn "  不要了  → git checkout -- <上面列出的文件>"
  warn "在此之前，本脚本不会改任何东西。"
  exit 2
fi

if [ "$DRY" -eq 1 ]; then
  echo
  info "--dry-run：没有做任何修改。"
  info "去掉 --dry-run 会设置 core.fileMode=false（不动文件内容）。"
  exit 0
fi

echo
if [ "$(git config --get core.fileMode || echo true)" != "false" ]; then
  git config core.fileMode false
  info "已设置 core.fileMode=false —— git 不再把可执行位当改动（以后不会再因此冲突）"
else
  info "core.fileMode 已经是 false"
fi

if [ "$DISCARD" -eq 1 ] && [ ${#MODE_ONLY[@]} -gt 0 ]; then
  for f in "${MODE_ONLY[@]}"; do
    git checkout -- "$f" 2>/dev/null || true
  done
  info "已丢弃 ${#MODE_ONLY[@]} 个纯 mode 改动（内容本来就一样）"
fi

echo
info "现在可以拉了："
echo "    git pull"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    colcon build --packages-select lio_sam_bench lio_sam_gz_bringup"
echo
left=$(git status --porcelain | wc -l)
if [ "$left" -eq 0 ]; then
  info "工作区已干净（0 个改动）"
else
  warn "还剩 $left 个改动；如果 pull 仍被拒，用 --discard 丢弃纯 mode 的那些"
fi
