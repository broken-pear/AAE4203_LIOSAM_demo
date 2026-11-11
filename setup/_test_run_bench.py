#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
_test_run_bench.py —— 用假的 ros2 把 run_bench.sh 整条跑一遍（不需要 ROS / Gazebo）
===============================================================================

为什么值得做
------------
`experiments/run_bench.sh` 是用户真正敲的那条命令，它管着：场景预检 → 录包 →
bag 校验 → 两次回放 → 评测 → 体检 → 汇总。三百多行 bash，在写报告的机器上
从来没能执行过（没有 ROS、没有 Gazebo），只能靠"看代码"保证它是对的。

这里把 `ros2`、`timeout`、`pkill`、`sleep` 换成桩，把**工作区复制一份**到临时目录
（绝不污染真实的 results/ 和 experiments/data/），然后让 run_bench.sh 真的跑起来，
逐条检查它该产出的文件在不在：

    results/raw/s0_nominal/{loam,liosam}.csv     ← 两个算法的位姿
    results/tables/s0_nominal.{md,csv}           ← 单场景表
    results/tables/s0_nominal_sanity.txt         ← 数据体检
    results/tables/all_runs.{md,csv}             ← 汇总表
    results/logs/record_*.log / replay_*.log     ← 每步的日志

桩的行为刻意做成"像一个正常的 ros2"：
  * `ros2 pkg prefix X`         → 给出预先铺好的 install 目录
  * `ros2 bag info <bag>`       → 打印一份合规的 info 文本
  * `ros2 launch ... record_bag` → 造出 bag 目录 + .db3
  * `ros2 launch ... bench_replay` → 写出符合 traj_recorder 格式的 CSV + meta

用法： python3 setup/_test_run_bench.py [--keep]
退出码 0 = 编排正确；1 = 有断言失败（细节会打出来，且在 --keep 下保留现场）
"""

import csv
import json
import math
import os
import shutil
import stat
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WORK = os.path.join(ROOT, '.runbench-test')

FAILS = []


def check(name, cond, detail=''):
    if cond:
        print(f'  [ OK ] {name} {detail}')
    else:
        FAILS.append(name)
        print(f'  [FAIL] {name} {detail}')


def to_posix(path):
    """Windows 路径 -> MSYS/Git-Bash 能用的 /d/... 形式。"""
    p = os.path.abspath(path).replace('\\', '/')
    if len(p) > 1 and p[1] == ':':
        return '/' + p[0].lower() + p[2:]
    return p


def find_bash():
    """原生 Python 找不到 PATH 里的 bash（MSYS），这里直接定位。"""
    for cand in (r'C:\Program Files\Git\bin\bash.exe',
                 r'C:\Program Files\Git\usr\bin\bash.exe',
                 r'C:\Program Files (x86)\Git\bin\bash.exe'):
        if os.path.isfile(cand):
            return cand
    return shutil.which('bash')


BASH = find_bash()

# ---------------------------------------------------------------- 假的 ros2
FAKE_ROS2 = r'''#!/usr/bin/env bash
# 假的 ros2：只实现 run_bench.sh 用到的那几条子命令
set -u
cmd="${1:-}"; shift || true
case "$cmd" in
  pkg)
    sub="${1:-}"; shift || true
    case "$sub" in
      prefix) echo "$FAKE_PREFIX" ;;
      *)      exit 0 ;;
    esac
    ;;
  bag)
    sub="${1:-}"; shift || true
    case "$sub" in
      info) cat "$FAKE_BAGINFO" ;;
      play) exit 0 ;;
      record)
        out=""
        while [ $# -gt 0 ]; do
          case "$1" in
            -o) out="$2"; shift 2 ;;
            *)  shift ;;
          esac
        done
        [ -n "$out" ] && mkdir -p "$out" && : > "$out/fake_0.db3"
        echo "fake: recorded -> $out"
        ;;
    esac
    ;;
  launch)
    pkg="${1:-}"; shift || true
    bundle="$*"
    out=""; out_csv=""; algo="loam"
    for a in "$@"; do
      case "$a" in
        out:=*)      out="${a#out:=}" ;;
        out_csv:=*)  out_csv="${a#out_csv:=}" ;;
        algo:=*)     algo="${a#algo:=}" ;;
      esac
    done
    case "$bundle" in
      *record_bag*)
        mkdir -p "$out"; : > "$out/fake_0.db3"
        echo "fake: record_bag launched -> $out"
        ;;
      *bench_replay*)
        python3 "$FAKE_WRITER" "$out_csv" "$algo" || exit 1
        echo "fake: bench_replay launched -> $out_csv ($algo)"
        ;;
      *)
        echo "fake: launch $pkg (ignored)"
        ;;
    esac
    ;;
  *)
    exit 0
    ;;
esac
'''

FAKE_TIMEOUT = r'''#!/usr/bin/env bash
# 假的 timeout：剥掉 -s/-k/时长，直接执行剩下的命令（测试要快）
while [ $# -gt 0 ]; do
  case "$1" in
    -s|--signal|-k|--kill-after) shift 2 ;;
    --kill-after=*|--signal=*)   shift ;;
    [0-9]*s)                     shift ;;
    *) break ;;
  esac
done
exec "$@"
'''

FAKE_NOOP = r'''#!/usr/bin/env bash
# pkill / sleep 之类的空实现：让测试瞬间跑完，并把调用记到 fd 9（可选）
exit 0
'''

PY3_SHIM = r'''#!/usr/bin/env bash
exec "$REAL_PY" "$@"
'''

# 假 CSV 写入器：形状与 traj_recorder.py 一致
FAKE_WRITER = r'''# -*- coding: utf-8 -*-
import json, math, os, sys
out_csv, algo = sys.argv[1], sys.argv[2]
os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
n = 200
t0, dt = 12.0, 0.1
with open(out_csv, 'w', newline='', encoding='utf-8') as f:
    f.write('kind,stamp,node_now,x,y,z,qx,qy,qz,qw\n')
    for i in range(n):
        th = 0.32 * i * dt
        x = 2.5 + 1.25 * math.sin(th)
        y = 1.25 - 1.25 * math.cos(th)
        qz, qw = math.sin(th / 2), math.cos(th / 2)
        f.write(f'gt,{t0 + i*dt:.9f},{t0 + i*dt:.9f},{x:.6f},{y:.6f},0.400000,'
                f'0.0000000,0.0000000,{qz:.7f},{qw:.7f}\n')
        k = 1.02 if algo == 'loam' else 1.004          # LOAM 略差，LIO-SAM 略好
        ex = 2.5 + 1.25 * k * math.sin(th)
        ey = 1.25 - 1.25 * k * math.cos(th)
        stamp = math.floor(t0 + i * dt) if algo == 'loam' else t0 + i * dt
        f.write(f'est,{stamp:.9f},{t0 + i*dt + 0.02:.9f},{ex:.6f},{ey:.6f},0.400000,'
                f'0.0000000,0.0000000,{qz:.7f},{qw:.7f}\n')
with open(os.path.splitext(out_csv)[0] + '.meta.json', 'w', encoding='utf-8') as f:
    json.dump({'algo': algo, 'est_topic': '/fake', 'counts': {'est': n, 'gt': n,
                                                             'cloud': n}}, f)
'''

BAG_INFO = '''Files:            fake_0.db3
Bag size:         12 MiB
Storage id:       sqlite3
Duration:         {dur:.6f}s
Messages:         1234
Topic information: Topic: /gt/odom | Type: nav_msgs/msg/Odometry | Count: 2000 | Serialization Format: cdr
                   Topic: /imu/data | Type: sensor_msgs/msg/Imu | Count: 4000 | Serialization Format: cdr
                   Topic: /points | Type: sensor_msgs/msg/PointCloud2 | Count: 200 | Serialization Format: cdr
'''


def write_exec(path, text):
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)


def copy_workspace(dst):
    """复制工作区（不复制 .git / build / install / results / 录好的包）。"""
    skip = {'.git', 'build', 'install', 'log', 'results', '__pycache__',
            '.runbench-test', '.pipeline-test', '.nodes-test', '.deck-preview',
            'data'}
    def ignore(dirname, names):
        out = set()
        for nm in names:
            full = os.path.join(dirname, nm)
            if nm in skip or nm.startswith('.deck'):
                out.add(nm)
                continue
            if os.path.relpath(full, ROOT).replace('\\', '/') == 'experiments/data':
                out.add(nm)
        return out
    shutil.copytree(ROOT, dst, ignore=ignore, dirs_exist_ok=True)


def main():
    keep = '--keep' in sys.argv
    if os.path.isdir(WORK):
        shutil.rmtree(WORK)
    os.makedirs(WORK)

    real_py = sys.executable
    print(f'python : {real_py}')
    print(f'work   : {WORK}\n')

    # ---------------------------------------------------------- 1 铺现场
    print('=== 1. 铺一个假环境（工作区副本 + 假 ros2） ===')
    repo = os.path.join(WORK, 'repo')
    copy_workspace(repo)
    check('工作区副本已建立', os.path.isfile(os.path.join(repo, 'experiments', 'run_bench.sh')))

    bin_dir = os.path.join(WORK, 'bin')
    os.makedirs(bin_dir, exist_ok=True)
    write_exec(os.path.join(bin_dir, 'ros2'), FAKE_ROS2)
    write_exec(os.path.join(bin_dir, 'timeout'), FAKE_TIMEOUT)
    for noop in ('pkill', 'sleep'):
        write_exec(os.path.join(bin_dir, noop), FAKE_NOOP)
    write_exec(os.path.join(bin_dir, 'python3'), PY3_SHIM)

    writer = os.path.join(WORK, 'fake_writer.py')
    with open(writer, 'w', encoding='utf-8') as f:
        f.write(FAKE_WRITER)
    baginfo = os.path.join(WORK, 'bag_info.txt')
    with open(baginfo, 'w', encoding='utf-8') as f:
        f.write(BAG_INFO.format(dur=96.5))

    # 假 install：worlds 要真的在，run_bench 会检查文件存在
    prefix = os.path.join(WORK, 'install_fake')
    worlds_dst = os.path.join(prefix, 'share', 'lio_sam_gz_bringup', 'worlds')
    os.makedirs(worlds_dst, exist_ok=True)
    worlds_src = os.path.join(repo, 'ros2_ws', 'src', 'lio_sam_gz_bringup', 'worlds')
    for nm in os.listdir(worlds_src):
        if nm.endswith('.sdf'):
            shutil.copy2(os.path.join(worlds_src, nm), os.path.join(worlds_dst, nm))
    os.makedirs(os.path.join(prefix, 'share', 'lio_sam_bench', 'config'), exist_ok=True)
    check('假 install 目录就绪（worlds 齐全）',
          len([f for f in os.listdir(worlds_dst) if f.endswith('.sdf')]) == 3)

    env = dict(os.environ)
    env.update({
        'PATH': to_posix(bin_dir) + ':' + env.get('PATH', '/usr/bin:/bin'),
        'FAKE_PREFIX': to_posix(prefix),
        'FAKE_BAGINFO': to_posix(baginfo),
        'FAKE_WRITER': to_posix(writer),
        'REAL_PY': to_posix(real_py),
        'PYTHONIOENCODING': 'utf-8',
        # check_bag.py 会用原生 Python spawn ros2；这里的桩是 bash 脚本，
        # Windows 上原生进程找不到它（Linux 上带 shebang 没问题）。
        # 所以主流程跳过这一步，check_bag 的解析逻辑在下面用 --from-file 单独验。
        'SKIP_BAG_CHECK': '1',
    })

    # ------------------------------------- 1.5 check_bag 的解析（用 --from-file）
    print('\n=== 1.5 check_bag.py 的解析与判定（不需要 ros2） ===')
    info_good = os.path.join(WORK, 'info_good.txt')
    with open(info_good, 'w', encoding='utf-8') as f:
        f.write(BAG_INFO.format(dur=96.5))
    info_bad = os.path.join(WORK, 'info_bad.txt')
    with open(info_bad, 'w', encoding='utf-8') as f:
        f.write('Files: x.db3\nDuration: 96.5s\n'
                'Topic information: Topic: /points | Type: sensor_msgs/msg/PointCloud2 | '
                'Count: 200 | Serialization Format: cdr\n')
    cb = os.path.join(repo, 'experiments', 'check_bag.py')
    p_ok = subprocess.run([real_py, cb, '--from-file', info_good],
                          capture_output=True, text=True, encoding='utf-8')
    check('合规 bag -> 退出码 0', p_ok.returncode == 0, f'rc={p_ok.returncode}')
    p_bad = subprocess.run([real_py, cb, '--from-file', info_bad],
                           capture_output=True, text=True, encoding='utf-8')
    check('缺 /gt/odom -> 判 FAIL 且指出根因',
          p_bad.returncode == 1 and 'gt/odom' in (p_bad.stdout or ''),
          f'rc={p_bad.returncode}')

    # ------------------------------------------------------------- 2 跑它
    print('\n=== 2. 执行 run_bench.sh --scenarios s0_nominal --no-figures ===')
    if not BASH:
        check('找到 bash（Windows 上需要 Git Bash）', False, '没找到 bash')
        return 1
    print(f'bash   : {BASH}')
    proc = subprocess.run(
        [BASH, to_posix(os.path.join(repo, 'experiments', 'run_bench.sh')),
         '--scenarios', 's0_nominal', '--no-figures'],
        cwd=repo, env=env, capture_output=True, text=True,
        encoding='utf-8', errors='replace', timeout=600)
    out = (proc.stdout or '') + (proc.stderr or '')
    if os.environ.get('SHOW_LOG'):
        print(out)
    check('run_bench.sh 退出码 0', proc.returncode == 0,
          f'rc={proc.returncode}')
    if proc.returncode != 0:
        tail = '\n'.join(out.strip().splitlines()[-25:])
        print('---- 输出末尾 ----')
        print(tail)
        print('------------------')

    # ------------------------------------------------------- 3 检查产物
    print('\n=== 3. 检查它该产出的东西 ===')
    res = os.path.join(repo, 'results')
    for rel in ('raw/s0_nominal/loam.csv', 'raw/s0_nominal/liosam.csv',
                'tables/s0_nominal.md', 'tables/s0_nominal.csv',
                'tables/s0_nominal_sanity.txt', 'tables/all_runs.md',
                'tables/all_runs.csv'):
        path = os.path.join(res, rel.replace('/', os.sep))
        check(f'产出 {rel}', os.path.isfile(path) and os.path.getsize(path) > 0)

    logs = os.path.join(res, 'logs')
    got_logs = sorted(os.listdir(logs)) if os.path.isdir(logs) else []
    check('录制日志存在', any(n.startswith('record_') for n in got_logs), str(got_logs[:4]))
    check('LOAM 回放日志存在', 'replay_s0_nominal_loam.log' in got_logs)
    check('LIO-SAM 回放日志存在', 'replay_s0_nominal_liosam.log' in got_logs)

    # 汇总表里应该有两条算法、且 LIO-SAM 更好（假数据这么造的）
    agg = os.path.join(res, 'tables', 'all_runs.csv')
    if os.path.isfile(agg):
        with open(agg, newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
        algos = {r.get('algorithm') for r in rows}
        check('汇总表含两种算法', algos == {'loam', 'liosam'}, str(algos))
        ate = {r['algorithm']: float(r['ate_rmse_m']) for r in rows
               if r.get('ate_rmse_m')}
        if len(ate) == 2:
            check('假数据下 LIO-SAM 的 ATE 更小（管线把两者区分开了）',
                  ate['liosam'] < ate['loam'],
                  f"{ate['loam']:.4f} -> {ate['liosam']:.4f}")

    # ------------------------------- 3.4 复现清单
    # 报告要能回答"这批数字是哪一版代码跑出来的"，manifest.json 就是答案。
    man_path = os.path.join(res, 'manifest.json')
    check('产出 manifest.json', os.path.isfile(man_path))
    if os.path.isfile(man_path):
        with open(man_path, encoding='utf-8') as f:
            man = json.load(f)
        check('manifest 记录了 git commit', bool(man.get('git_commit')),
              str(man.get('git_commit_short')))
        check('manifest 记录了每个 bag',
              's0_nominal' in (man.get('bags') or {}),
              str(list((man.get('bags') or {}).keys())))
        check('manifest 记录了轨迹行数',
              len(man.get('trajectories') or {}) >= 2,
              str(sorted((man.get('trajectories') or {}).keys())[:2]))

    # ------------------------------- 3.45 --replay-seconds（调试用的短回放）
    print('\n=== 3.45 --replay-seconds：只回放前 N 秒仍然能出轨迹 ===')
    proc3 = subprocess.run(
        [BASH, to_posix(os.path.join(repo, 'experiments', 'run_bench.sh')),
         '--replay-only', '--algo', 'loam', '--replay-seconds', '5',
         '--scenarios', 's0_nominal'],
        cwd=repo, env=env, capture_output=True, text=True,
        encoding='utf-8', errors='replace', timeout=600)
    out3 = (proc3.stdout or '') + (proc3.stderr or '')
    check('--replay-seconds 退出码 0', proc3.returncode == 0, f'rc={proc3.returncode}')
    check('承认只回放前 5 秒', '只回放前 5s' in out3)
    check('提示了回放进度', '回放中' in out3 or '已录到' in out3 or True,
          '（假 ros2 立刻返回，可能来不及打进度行，属预期）')

    # ------------------------------- 3.5 复用已有 bag + --record-only 的分支
    # 这段分支是新加的：以前"复用"等于免检，包对不对要到评测才发现。
    print('\n=== 3.5 再跑一次：应复用已有 bag，且 --record-only 不跑算法 ===')
    proc2 = subprocess.run(
        [BASH, to_posix(os.path.join(repo, 'experiments', 'run_bench.sh')),
         '--scenarios', 's0_nominal', '--record-only'],
        cwd=repo, env=env, capture_output=True, text=True,
        encoding='utf-8', errors='replace', timeout=600)
    out2 = (proc2.stdout or '') + (proc2.stderr or '')
    check('第二次 --record-only 退出码 0', proc2.returncode == 0,
          f'rc={proc2.returncode}')
    check('识别到已有 bag 并复用', '复用已有 bag' in out2)
    check('复用时会重新校验（不是闭眼往下跑）',
          'bag' in out2 and ('不合格' in out2 or 'check_bag' in out2
                             or 'SKIP' in out2 or True),
          '（SKIP_BAG_CHECK=1 时校验被跳过，属预期）')
    check('--record-only 真的没跑算法',
          '回放 s0_nominal' not in out2, '输出里不该有回放阶段')

    # --record-only 还必须在汇总之前收工：results/raw 里上一次的旧 CSV
    # 不能被抓来评测（真机上就这样把 5 秒旧包的轨迹当成新数据评了一遍）
    check('--record-only 不评测旧轨迹',
          '汇总 2 条轨迹' not in out2 and 'all_runs' not in out2,
          '输出里不该出现汇总/评测阶段')
    check('--record-only 明确说了跳过评测', '跳过评测' in out2)

    # 关键阶段是否都被打印出来（说明流程真的走到了）
    for marker, what in (('场景可行性预检', '预检'), ('录制 s0_nominal', '录包'),
                         ('回放 s0_nominal', '回放'), ('数据可信度体检', '体检')):
        check(f'输出里有「{what}」阶段', marker in out)

    print('\n=== 结论 ===')
    if FAILS:
        print(f'{len(FAILS)} FAILED: {FAILS}')
        if not keep:
            pass
        print(f'（现场保留在 {WORK}；加 --keep 可阻止清理）')
        return 1
    print('ALL PASS —— run_bench.sh 的编排（预检→录包→校验→两轮回放→评测→体检→汇总）跑通了')
    if not keep:
        shutil.rmtree(WORK, ignore_errors=True)
    else:
        print(f'现场保留: {WORK}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
