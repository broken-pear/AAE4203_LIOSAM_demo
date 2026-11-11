# =============================================================================
# write_manifest.sh —— 把这批实验的"出处"写成 results/manifest.json
#
# 报告里的每个数字都要能回答：哪一版代码、哪台机器、什么时候、用了哪些场景、
# 包里有多少数据。这个脚本就是那个答案，也是复现的唯一入口。
#
# 由 run_bench.sh 结尾调用，也可以单独跑：
#   bash experiments/write_manifest.sh [仓库根目录]
# =============================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${1:-$(cd "$HERE/.." && pwd)}"
cd "$ROOT" || { echo "[error] 进不去 $ROOT"; exit 1; }

PY="${PYTHON:-python3}"
command -v "$PY" >/dev/null 2>&1 || { echo "[error] 找不到 $PY"; exit 1; }

"$PY" - "$ROOT" <<'PYEOF'
import datetime
import glob
import json
import os
import subprocess
import sys


root = sys.argv[1]


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=20).stdout.strip()
    except Exception:                                          # noqa: BLE001
        return ''


def count_rows(path):
    try:
        with open(path, encoding='utf-8') as f:
            return max(0, sum(1 for _ in f) - 1)
    except Exception:                                          # noqa: BLE001
        return 0


man = {
    'generated': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
    'git_commit': sh('git rev-parse HEAD'),
    'git_commit_short': sh('git rev-parse --short HEAD'),
    'git_subject': sh('git log -1 --format=%s'),
    'git_dirty': bool(sh('git status --porcelain')),
    'host': sh('hostname'),
    'kernel': sh('uname -sr'),
    'python': sys.version.split()[0],
    'ros_distro': os.environ.get('ROS_DISTRO', ''),
    'bags': {},
    'trajectories': {},
    'tables': {},
}

# 每个包：文件名 + 大小（能看出是不是空的）
for d in sorted(glob.glob(os.path.join(root, 'experiments', 'data', '*'))):
    if not os.path.isdir(d):
        continue
    files = sorted(glob.glob(os.path.join(d, '*.db3')) +
                   glob.glob(os.path.join(d, '*.mcap')))
    man['bags'][os.path.basename(d)] = {
        'files': [os.path.basename(f) for f in files],
        'bytes': sum(os.path.getsize(f) for f in files),
    }

# 每条轨迹：行数（est/gt 各多少）
for f in sorted(glob.glob(os.path.join(root, 'results', 'raw', '*', '*.csv'))):
    rows = count_rows(f)
    man['trajectories'][os.path.relpath(f, root).replace('\\', '/')] = {'rows': rows}

# 每张表
for f in sorted(glob.glob(os.path.join(root, 'results', 'tables', '*.md')) +
                glob.glob(os.path.join(root, 'results', 'tables', '*.csv')) +
                glob.glob(os.path.join(root, 'results', 'tables', '*.txt'))):
    man['tables'][os.path.basename(f)] = os.path.getsize(f)

# 体检结论（有 FAIL 就在清单里标出来 —— 别让人拿坏数据写报告）
sanity = os.path.join(root, 'results', 'tables', 'sanity.txt')
if os.path.isfile(sanity):
    txt = open(sanity, encoding='utf-8', errors='replace').read()
    man['sanity_has_fail'] = 'FAIL' in txt
    man['sanity_has_warn'] = 'WARN' in txt

out = os.path.join(root, 'results', 'manifest.json')
os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out, 'w', encoding='utf-8') as f:
    json.dump(man, f, indent=2, ensure_ascii=False)

print(f"[manifest] {os.path.relpath(out, root)}")
print(f"[manifest] commit {man['git_commit_short'] or '?'}"
      f"{'（工作区有未提交改动）' if man['git_dirty'] else ''}")
print(f"[manifest] {len(man['bags'])} 个 bag / {len(man['trajectories'])} 条轨迹"
      f" / {len(man['tables'])} 个表")
PYEOF
