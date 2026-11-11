#!/usr/bin/env bash
# =============================================================================
# run_bench.sh —— LOAM vs LIO-SAM 公平对比实验：一键跑完"录包 → 回放 → 评测 → 出表"
#
# 用法：
#   bash experiments/run_bench.sh --list                 # 看有哪些场景
#   bash experiments/run_bench.sh                        # 全部场景 × 两种算法
#   bash experiments/run_bench.sh --scenarios s0_nominal s3_corridor
#   bash experiments/run_bench.sh --algo loam            # 只跑 baseline
#   bash experiments/run_bench.sh --replay-only          # 复用已有 bag，不重录
#   bash experiments/run_bench.sh --record-only          # 只录包，不跑算法（调试录制链）
#   bash experiments/run_bench.sh --record               # 强制重录（覆盖）            
#   bash experiments/run_bench.sh --rate 0.7             # Jetson 跟不上时降速回放
#   bash experiments/run_bench.sh --repeats 3            # 每个场景录 3 份，看稳定性
#
# 一次"场景 × 算法"的流程：
#   1) 录包   experiments/data/<scenario>/             （Gazebo 跑一遍，录原始传感器+真值）
#   2) 回放   results/raw/<scenario>/<algo>.csv        （同一份包喂给算法，录位姿+真值）
#   3) 评测   results/json|tables/<scenario>.*         （ATE / RPE / 终点 / 鲁棒性）
#   4) 汇总   results/tables/all_runs.{md,csv} + results/figures/*.png
#
# 为什么先录包再回放，而不是"跑两次 Gazebo"：见 launch/bench_replay.launch.py 的注释。
# 一句话 —— 回放同一份包，两条算法看到的点云流逐字节相同，差异只能来自算法。
#
# 环境要求：已经 source 过 ROS 2 + 本工作空间（能 ros2 pkg prefix lio_sam_bench）
#
# 环境变量：
#   SKIP_BAG_CHECK=1   跳过录制后的 bag 校验（默认会校验话题/条数，尤其是 /gt/odom）
# =============================================================================
set -uo pipefail

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}==>${NC} $*"; }
warn() { echo -e "${YELLOW}[warn]${NC} $*"; }
die()  { echo -e "${RED}[error]${NC} $*"; exit 1; }

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCEN_DIR="$ROOT/experiments/scenarios"
DATA_DIR="$ROOT/experiments/data"
RAW_DIR="$ROOT/results/raw"
TABLES_DIR="$ROOT/results/tables"
JSON_DIR="$ROOT/results/json"
LOG_DIR="$ROOT/results/logs"
EVAL="$ROOT/ros2_ws/src/lio_sam_bench/scripts/evaluate.py"
FIGS="$ROOT/ros2_ws/src/lio_sam_bench/scripts/make_figures.py"

ALL_SCENARIOS=(s0_nominal s1_fast_yaw s2_high_speed s3_corridor s4_sparse_hall s5_long_run)
SCENARIOS=()
ALGOS=(loam liosam)
RATE="1.0"
REPLAY_SECONDS=0        # >0 时只回放前 N 秒（调试用，省时间）
REPEATS=1
FORCE_RECORD=0
REPLAY_ONLY=0
RECORD_ONLY=0
SKIP_FIGURES=0

# ---------------------------------------------------------------- 参数解析
while [ $# -gt 0 ]; do
  case "$1" in
    --list)        LIST=1; shift; continue ;;
    --scenarios)
      shift
      while [ $# -gt 0 ] && [[ "$1" != --* ]]; do SCENARIOS+=("$1"); shift; done
      continue
      ;;
    --algo)
      shift
      case "${1:-both}" in
        loam)   ALGOS=(loam) ;;
        liosam) ALGOS=(liosam) ;;
        both)   ALGOS=(loam liosam) ;;
        *) die "--algo 只支持 loam / liosam / both" ;;
      esac
      shift
      continue
      ;;
    --rate)        RATE="${2:?--rate 需要数值}"; shift 2; continue ;;
    --replay-seconds)
      REPLAY_SECONDS="${2:?--replay-seconds 需要数值}"; shift 2; continue ;;
    --repeats)     REPEATS="${2:?--repeats 需要数值}"; shift 2; continue ;;
    --record)      FORCE_RECORD=1; shift; continue ;;
    --replay-only) REPLAY_ONLY=1; shift; continue ;;
    --record-only) RECORD_ONLY=1; shift; continue ;;
    --no-figures)  SKIP_FIGURES=1; shift; continue ;;
    -h|--help)     sed -n '2,40p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "不认识的参数: $1（--help 看用法）" ;;
  esac
done

[ ${#SCENARIOS[@]} -eq 0 ] && SCENARIOS=("${ALL_SCENARIOS[@]}")

# ---------------------------------------------------------------- 前置检查
require_ros() {
  command -v ros2 >/dev/null 2>&1 || die "没找到 ros2 —— 先 source /opt/ros/jazzy/setup.bash"
  ros2 pkg prefix lio_sam_bench >/dev/null 2>&1 || \
    die "找不到 lio_sam_bench 包 —— 先 source ros2_ws/install/setup.bash 并 colcon build"
  ros2 pkg prefix lio_sam_gz_bringup >/dev/null 2>&1 || \
    die "找不到 lio_sam_gz_bringup 包 —— 先 colcon build"
  python3 -c 'import yaml' 2>/dev/null || die "缺 python3-yaml：sudo apt install python3-yaml"
}

# 从场景 yaml 里取一个字段（scenario.ros__parameters 段）
# 为什么要套一层 ros__parameters：这个文件同时被当成 ROS 2 参数文件传给
# scripted_driver。ROS 的参数解析器要求每个顶层节点名下面必须有 ros__parameters，
# 否则整份文件可能被判非法 —— 那会让所有录制直接起不来。
scenario_field() {
  python3 - "$1" "$2" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as f:
    d = yaml.safe_load(f) or {}
sc = d.get('scenario', {}) or {}
sc = sc.get('ros__parameters', sc) or {}      # 兼容老格式（没有 ros__parameters）
print(sc.get(sys.argv[2], ''))
PY
}

# 清场：上一次跑残留的 gz / 桥接 / 算法进程会让第二次实验失真
# （典型现象：两个 /clock 源，RViz 一直 "jump back in time"）
cleanup_processes() {
  # 注意：这里的模式要足够具体，别把自己或用户的 shell 杀掉。
  # （用 'lio_sam_' 会误伤命令行里带工作空间路径的进程，所以列全名）
  for pat in 'gz-sim' 'gz sim' 'parameter_bridge' 'sensor_prep.py' \
             'patrol_driver.py' 'scripted_driver.py' \
             'alaserMapping' 'alaserOdometry' 'ascanRegistration' \
             'lio_sam_imuPreintegration' 'lio_sam_imageProjection' \
             'lio_sam_featureExtraction' 'lio_sam_mapOptimization' \
             'gt_sync.py' 'traj_recorder.py' \
             'ros2 bag record' 'ros2 bag play'; do
    pkill -INT -f "$pat" 2>/dev/null
  done
  sleep 2
  pkill -KILL -f 'gz-sim' 2>/dev/null
  return 0
}

bag_duration() {   # 秒（浮点）；解析失败返回空
  ros2 bag info "$1" 2>/dev/null \
    | awk -F': *' '/^Duration/ {print $2}' | sed 's/s$//' | head -1
}

bag_exists() {
  # 不要只认 .db3：rosbag2 的默认存储格式在不同发行版里是 sqlite3 或 mcap，
  # 只查一种就会把"录成功但扩展名不同"误判成"没录到"。
  [ -d "$1" ] && { compgen -G "$1/*.db3" >/dev/null 2>&1 \
                   || compgen -G "$1/*.mcap" >/dev/null 2>&1; }
}

bag_files() {   # 列出包里的存储文件（诊断用）
  ls -1 "$1"/*.db3 "$1"/*.mcap 2>/dev/null | xargs -r -n1 basename 2>/dev/null | tr '\n' ' '
}

# --list：函数都定义完了才处理，避免"先调用后定义"
if [ "${LIST:-0}" -eq 1 ]; then
  echo "可用场景（experiments/scenarios/*.yaml）："
  for s in "${ALL_SCENARIOS[@]}"; do
    f="$SCEN_DIR/$s.yaml"
    [ -f "$f" ] || { echo "  $s  (缺少 $f)"; continue; }
    desc="$(scenario_field "$f" description)"
    secs="$(scenario_field "$f" record_seconds)"
    wld="$(scenario_field "$f" world)"
    # 解析不出来就显式写 '?'，不要让表格出现空列 ——
    # 空列看起来像"这个场景没有时长"，实际是 python3/yaml 没就位。
    [ -n "$secs" ] || secs="?"
    [ -n "$wld" ] || wld="?"
    [ -n "$desc" ] || desc="（读不出 description：检查 python3-yaml）"
    # 顺带显示这个场景的包录了没有 —— 跑长实验时一眼看到进度
    bag="$DATA_DIR/$s"
    if bag_exists "$bag"; then
      st="已录 $(bag_duration "$bag")s"
    else
      st="未录"
    fi
    printf '  %-16s %-18s %6ss  %-12s %s\n' "$s" "$wld" "$secs" "$st" "$desc"
  done
  exit 0
fi

write_manifest() {
  if [ -f "$ROOT/experiments/write_manifest.sh" ]; then
    bash "$ROOT/experiments/write_manifest.sh" "$ROOT"
  else
    return 1
  fi
}

# 校验一个 bag 能不能用。录完要验，**复用已有 bag 时同样要验** ——
# 否则"复用"等于闭着眼睛往下跑，问题会推迟到评测阶段才炸出来，
# 而那时已经花掉了两轮回放的时间。
validate_bag() {
  local tag="$1" bag="$2"
  [ "${SKIP_BAG_CHECK:-0}" = "1" ] && return 0
  if ! python3 "$ROOT/experiments/check_bag.py" "$bag"; then
    warn "bag 不合格 —— 跳过 ${tag} 的回放（修好后用 --record 重录）"
    return 1
  fi
  return 0
}

# ---------------------------------------------------------------- 1) 录包
record_one() {
  local name="$1" tag="$2"
  local yaml="$SCEN_DIR/$name.yaml"
  local world_field secs bag
  world_field="$(scenario_field "$yaml" world)"
  secs="$(scenario_field "$yaml" record_seconds)"
  [ -n "$secs" ] || { warn "$yaml 里没有 scenario.record_seconds"; return 1; }
  bag="$DATA_DIR/${name}${tag}"

  local world_path="$world_field"
  case "$world_field" in
    /*) : ;;
    *)  world_path="$(ros2 pkg prefix lio_sam_gz_bringup)/share/lio_sam_gz_bringup/worlds/$world_field" ;;
  esac
  [ -f "$world_path" ] || { warn "找不到 world: $world_path"; return 1; }

  cleanup_processes
  rm -rf "$bag"
  # 重录同一个场景时，把上一次的算法轨迹一起清掉。
  # 否则新包回放失败时，旧的 CSV 会留在 results/raw 里被汇总进去，
  # 得到一张"一半新一半旧"的表 —— 真机上已经发生过一次（旧 5 秒包的数据
  # 被当成本次结果评测）。宁可空，也不要混。
  rm -rf "$RAW_DIR/${name}${tag}"
  mkdir -p "$LOG_DIR"

  log "录制 ${name}${tag}  ${secs}s  world=$(basename "$world_path")"
  log "  日志: ${LOG_DIR}/record_${name}${tag}.log"
  # SIGINT 让 rosbag2 正常收尾（写 metadata.yaml）；超时后 20 s 强杀
  timeout -s INT --kill-after=20 "${secs}s" \
    ros2 launch lio_sam_bench record_bag.launch.py \
      world:="$world_path" \
      driver_params:="$yaml" \
      out:="$bag" \
      > "$LOG_DIR/record_${name}${tag}.log" 2>&1
  local rc=$?
  # 124 = 被 timeout 打断（预期）；130 = Ctrl-C
  if [ "$rc" -ne 0 ] && [ "$rc" -ne 124 ] && [ "$rc" -ne 130 ]; then
    warn "录制退出码 $rc（看看 ${LOG_DIR}/record_${name}${tag}.log）"
  fi
  cleanup_processes

  if ! bag_exists "$bag"; then
    warn "录制失败：$bag 里没有可用的存储文件（.db3 / .mcap 都没有）"
    # 先把目录里到底有什么打出来 —— "文件其实在、只是扩展名不同"这种情况
    # 光看一句"没有 .db3"是分辨不出来的
    if [ -d "$bag" ]; then
      warn "目录内容："
      ls -la "$bag" 2>/dev/null | sed 's/^/    /'
    else
      warn "目录压根没被创建：$bag"
    fi
    warn "日志前 30 行（第一条红字通常就在里面）:"
    head -30 "${LOG_DIR}/record_${name}${tag}.log" 2>/dev/null | sed 's/^/    /'
    echo
    warn "常见原因，按出现频率排："
    warn "  1) 有话题一直没出现 → rosbag2 会一直等（它要先查清每个话题的消息类型）"
    warn "     ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py"
    warn "     ros2 topic hz /points_raw   # 应 ≈10 Hz"
    warn "     ros2 topic hz /gt/odom      # 应 ≈100 Hz（真值插件）"
    warn "  2) 没有图形界面（SSH）→ gpu_lidar 不渲染，点云一条不发"
    warn "  3) 看上面 watchdog 打的那张表：哪一行 [BAD] 就是哪一段断的"
    return 1
  fi

  # 录完立刻验：话题齐不齐、条数够不够。
  # 最坑的情况是 /gt/odom 一条都没有 —— bag 照写、算法照跑，
  # 直到评测才报"gt 为空"。这里提前拦住，并给出根因。
  validate_bag "$name$tag" "$bag" || return 1
  log "  录好了: $(bag_duration "$bag") s"
  return 0
}

# ---------------------------------------------------------------- 2) 回放 + 录制算法位姿
replay_one() {
  local name="$1" tag="$2" algo="$3"
  local bag="$DATA_DIR/${name}${tag}"
  local out="$RAW_DIR/${name}${tag}/${algo}.csv"
  local dur budget dur_int
  dur="$(bag_duration "$bag")"
  [ -n "${dur:-}" ] || dur="120"
  # 回放预算给足余量：bag 正常放完 launch 会自己退出，这里的超时只是兜底。
  dur_int="${dur%.*}"
  case "$dur_int" in
    ''|*[!0-9]*) dur_int=180 ;;
  esac
  budget=$(( dur_int * 3 / 2 + 60 ))
  # 调试时只回放前 N 秒（--replay-seconds）：改算法/改参数时不必每次等整个包
  if [ "${REPLAY_SECONDS:-0}" -gt 0 ] 2>/dev/null; then
    budget="$REPLAY_SECONDS"
    warn "只回放前 ${budget}s（--replay-seconds），用于快速验证算法是否出数据"
  fi

  cleanup_processes
  mkdir -p "$(dirname "$out")" "$LOG_DIR"
  rm -f "$out" "${out%.csv}.meta.json"

  log "回放 ${name}${tag}  algo=${algo}  rate=${RATE}  budget=${budget}s"
  log "  输出写到日志，下面每 10 秒报一次进度（帧数不动 = 算法没出位姿）"
  # bag 放完 launch 会自己退出（见 bench_replay.launch.py 的 on_exit 处理器）
  timeout -s INT --kill-after=25 "${budget}s" \
    ros2 launch lio_sam_bench bench_replay.launch.py \
      bag:="$bag" algo:="$algo" out_csv:="$out" rate:="$RATE" \
      > "$LOG_DIR/replay_${name}${tag}_${algo}.log" 2>&1 &
  local lpid=$!
  local t0 last_n now_n el tick=0
  t0="$(date +%s)"
  last_n=-1
  # 等待期间每 10 秒打一行：既证明没卡死，也能看出算法是"一直不出"还是"中途停了"。
  # 内部 2 秒一探，这样 launch 结束时能在 2 秒内收尾，不必白等满 10 秒。
  while kill -0 "$lpid" 2>/dev/null; do
    sleep 2
    kill -0 "$lpid" 2>/dev/null || break
    tick=$(( tick + 1 ))
    [ $(( tick % 5 )) -eq 0 ] || continue
    el=$(( $(date +%s) - t0 ))
    now_n="$(awk -F, '$1=="est"' "$out" 2>/dev/null | wc -l)"
    if [ "$now_n" -gt "$last_n" ]; then
      log "    ${el}s/${budget}s  已录到 ${now_n} 帧"
    else
      log "    ${el}s/${budget}s  仍是 ${now_n} 帧（算法还没出位姿，见 ${LOG_DIR}/replay_${name}${tag}_${algo}.log）"
    fi
    last_n="$now_n"
  done
  wait "$lpid" 2>/dev/null
  local rc=$?
  if [ "$rc" -ne 0 ] && [ "$rc" -ne 124 ] && [ "$rc" -ne 130 ]; then
    warn "回放退出码 $rc（看看 ${LOG_DIR}/replay_${name}${tag}_${algo}.log）"
  fi
  cleanup_processes

  if [ ! -s "$out" ]; then
    warn "没生成 $out —— 算法可能没起来，看日志："
    tail -20 "$LOG_DIR/replay_${name}${tag}_${algo}.log" 2>/dev/null | sed 's/^/    /'
    return 1
  fi
  local n
  n="$(awk -F, '$1=="est"' "$out" | wc -l)"


  log "  录到 $n 帧算法位姿"
  if [ "$n" -lt 10 ]; then
    warn "只有 $n 帧，太少 —— 直接看日志里的证据："
    local rlog="$LOG_DIR/replay_${name}${tag}_${algo}.log"
    if [ -f "$rlog" ]; then
      warn "  关键行（error / fail / not enough / terminate）："
      grep -inE 'error|fail|not enough|terminate|exception|what\(\)|segmentation|died' "$rlog" \
        | head -12 | sed 's/^/      /' || true
      echo
      # 症状计数：有些"病因"是**重复的 INFO/WARN**，不在 error 里，例如
      # "Waiting for IMU data ..." —— 它每来一帧点云就打一次，重复几千次
      # 就等于"卡在 imageProjection"。只 grep error 会完全看不到它。
      warn "  症状计数（同一行重复上千次 = 卡在那里）："
      local pat cnt
      for pat in 'Waiting for IMU data' 'Not enough features' \
                 'unconnected trees' 'extrapolation' 'Invalid quaternion' \
                 'Point cloud is not in dense' 'Invalid sensor type'; do
        cnt="$(grep -c "$pat" "$rlog" 2>/dev/null || true)"
        [ -n "${cnt:-}" ] || cnt=0
        if [ "$cnt" -gt 0 ]; then
          printf '      %-32s × %s\n' "$pat" "$cnt"
        fi
      done
      echo
      warn "  日志末尾 15 行："
      tail -15 "$rlog" | sed 's/^/      /'
      echo
      # LIO-SAM 是四段式流水线：IMU 预积分 -> 图像投影 -> 特征提取 -> 建图。
      # mapOptimization 的源码里，只要 cloud_info 到达就会 publishOdometry()，
      # 所以"0 帧"必然是 cloud_info 根本没到 —— 卡在它前面某一段。
      # 这几行能直接指出卡在哪。
      if [ "$algo" = "liosam" ]; then
        warn "  LIO-SAM 各段是否起来（卡在哪一段一目了然）："
        local ms
        for ms in 'IMU Preintegration' 'Image Projection Started' \
                  'Feature Extraction Started' 'Map Optimization Started'; do
          if grep -qi "$ms" "$rlog" 2>/dev/null; then
            echo "      [OK  ] $ms"
          else
            echo "      [MISS] $ms"
          fi
        done
        echo
        warn "  判读：Map Optimization 起来了却没有帧 -> 上游没发出 cloud_info"
        warn "        卡在 Image Projection -> 点云/IMU 消息同步失败（队列或时间戳）"
        warn "        全都没起来 -> 节点启动即崩，看上面第一条 error"
      fi
    else
      warn "  找不到 $rlog"
    fi
    echo
    warn "  其它可能："
    warn "    - 算法节点启动即退出（参数名/文件不对）→ 上面第一条 error 就是原因"
    warn "    - 估计位姿话题名不对 → 对照 bench_replay.launch.py 顶部的 EST_TOPIC"
  fi
  return 0
}

# ---------------------------------------------------------------- 3) 评测
eval_one() {
  local name="$1" tag="$2"
  local dir="$RAW_DIR/${name}${tag}"
  local csvs=()
  local f
  for f in "$dir"/*.csv; do
    [ -s "$f" ] && csvs+=("$f")
  done
  if [ ${#csvs[@]} -eq 0 ]; then
    warn "没有可评测的 CSV（$dir）"
    return 1
  fi
  mkdir -p "$TABLES_DIR" "$JSON_DIR"
  python3 "$EVAL" --csv "${csvs[@]}" \
    --json-out "$JSON_DIR/${name}${tag}.json" \
    --md-out  "$TABLES_DIR/${name}${tag}.md" \
    --csv-out "$TABLES_DIR/${name}${tag}.csv"
  local rc=$?
  [ "$rc" -eq 0 ] || return "$rc"

  # 算完立刻怀疑一下：帧数够不够、两条算法是不是录到了同一个话题、
  # LIO-SAM 是不是反而更差（可能录错、也可能真是发现）。
  # 这里只提示、不阻断 —— 跑完整个矩阵再统一判断更省时间。
  python3 "$ROOT/experiments/sanity_check.py" "$TABLES_DIR/${name}${tag}.csv" \
    --json-dir "$JSON_DIR" 2>&1 | tee "$TABLES_DIR/${name}${tag}_sanity.txt" \
    || warn "可信度检查有 FAIL（见上）—— 这组数字先别写进报告"
  return 0
}


# 陈旧构建检查：源码/配置比 install/ 新，说明忘了 colcon build。
# 这个坑踩了两次：一次是 --storage sqlite3 没生效（录出来还是 .mcap），
# 一次是 baselinkFrame 改了但 install/ 里还是旧参数（TF 报错照旧）。
# 参数文件从 install/ 加载，所以"改了源码不重编"等于没改。
check_stale_build() {
  # 只报"装进去了、但比源码旧"的文件 —— 那才是真正需要重编的情况。
  #
  # 第一版比的是 ros2_ws/install 目录的 mtime（colcon 不更新它），编译成功也报警；
  # 第二版把所有"在源码里但不在安装清单里"的文件也算成陈旧（models/、worlds/
  # 之类本来就不用装），于是刚编译完就报"27 个未安装"。两次都是**误报**，
  # 而误报比不报更糟：它会让人怀疑刚刚成功的编译。
  # 现在分两类：
  #   stale   = install 里有对应文件，但源码更新  -> 警告（真的需要重编）
  #   absent  = install 里没有对应文件            -> 只在 DEBUG 里提示
  local src="$ROOT/ros2_ws/src" inst="$ROOT/ros2_ws/install"
  [ -d "$inst" ] || return 0

  local _pkg
  for _pkg in lio_sam_bench lio_sam_gz_bringup; do
    if [ ! -d "$inst/$_pkg" ]; then
      warn "ros2_ws/install/$_pkg 不存在 —— 这个包还没装进工作区。"
      warn "  注意：ROS 工作区是 ros2_ws/，colcon 必须在那个目录下跑。"
      warn "    cd ros2_ws"
      warn "    source /opt/ros/jazzy/setup.bash"
      warn "    colcon build --packages-select lio_sam_bench lio_sam_gz_bringup"
      return 0
    fi
  done

  local f rel pkg sub file cand stale=0 absent=0
  while IFS= read -r f; do
    rel="${f#$src/}"
    pkg="${rel%%/*}"
    rel="${rel#*/}"
    sub="${rel%%/*}"
    [ "$sub" = "$rel" ] && continue
    file="${rel#*/}"
    if [ "$sub" = "scripts" ]; then
      cand="$inst/$pkg/lib/$pkg/$file"
    else
      cand="$inst/$pkg/share/$pkg/$sub/$file"
    fi
    [ -L "$cand" ] && continue                # symlink-install：源码即产物
    if [ -e "$cand" ]; then
      if [ "$f" -nt "$cand" ]; then stale=$((stale + 1)); fi
    else
      absent=$((absent + 1))                  # 不在安装清单里，正常
    fi
  done < <(find "$src/lio_sam_bench" "$src/lio_sam_gz_bringup" -type f \
             \( -name '*.py' -o -name '*.yaml' -o -name '*.launch.py' \
                -o -name '*.rviz' -o -name '*.sdf' \) 2>/dev/null)

  if [ "$stale" -gt 0 ]; then
    warn "有 ${stale} 个已安装的文件比源码旧 —— 需要重新编译才能生效。"
    warn "    cd ros2_ws"
    warn "    source /opt/ros/jazzy/setup.bash"
    warn "    colcon build --packages-select lio_sam_bench lio_sam_gz_bringup"
  else
    log "install/ 与源码一致（另有 ${absent} 个源文件本就不需要安装）"
  fi
}


# ---------------------------------------------------------------- 主流程
require_ros
check_stale_build

# 图形环境提示：Gazebo 的 gpu_lidar 要渲染上下文，没有 DISPLAY 时常常一条点云都不发，
# 表现就是"录制 102 秒、包里一个 .db3 都没有"。提前说一句，省一次白等。
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  warn "没有 DISPLAY / WAYLAND_DISPLAY：Gazebo 的激光雷达可能不渲染，传感器就不发数据。"
  warn "  对策一：在桌面会话里跑（接显示器 / 远程桌面）"
  warn "  对策二：xvfb-run -a bash experiments/run_bench.sh --scenarios s0_nominal"
  warn "  对策三：export LIBGL_ALWAYS_SOFTWARE=1（软件渲染，慢但能出数据）"
  warn "  如果这套环境本来就能出点云，忽略这条即可。"
fi

# 开跑前先做一次离线可行性预检：不开 Gazebo，直接积分离散轨迹 + SAT 碰撞判定。
# 有场景会撞墙就直接停 —— 碰撞会让真值跳变，那一段数据既不能算算法的错，
# 也不能算算法的对，整个场景等于白录。
log "场景可行性预检（离线运动学 + 碰撞检测）"
python3 "$ROOT/experiments/check_scenarios.py" --scenarios "${SCENARIOS[@]}" \
  || die "有场景会撞墙/出界 —— 先改 experiments/scenarios/*.yaml，再回来录包"

log "场景: ${SCENARIOS[*]}"
log "算法: ${ALGOS[*]}   rate=$RATE   repeats=$REPEATS"
mkdir -p "$DATA_DIR" "$RAW_DIR" "$LOG_DIR"

# 预估总时长，先让人知道要等多久（录包 + 每个算法各回放一遍）。
# 只是估算：真机实时因子、是否复用已有 bag 都会影响。
if [ "$REPLAY_ONLY" -eq 0 ] || [ "$RECORD_ONLY" -eq 0 ]; then
  est=0
  for name in "${SCENARIOS[@]}"; do
    [ -f "$SCEN_DIR/$name.yaml" ] || continue
    secs="$(scenario_field "$SCEN_DIR/$name.yaml" record_seconds)"
    case "$secs" in ''|*[!0-9.]*) secs=100 ;; esac
    secs_int="${secs%.*}"
    [ -n "$secs_int" ] || secs_int=100
    need_record=0
    bag_exists "$DATA_DIR/$name" || need_record=1
    [ "$REPLAY_ONLY" -eq 1 ] && need_record=0
    n_algo=0
    [ "$RECORD_ONLY" -eq 0 ] && n_algo=$(( ${#ALGOS[@]} * REPEATS ))
    est=$(( est + need_record * (secs_int + 25) + n_algo * (secs_int + 20) ))
  done
  est_min=$(( (est + 59) / 60 ))
  [ "$est_min" -lt 1 ] && est_min=1
  log "预计总时长 ≈ ${est_min} 分钟（录包 + 回放 + 评测；实际取决于实时因子）"
fi

FAILED=()

for name in "${SCENARIOS[@]}"; do
  [ -f "$SCEN_DIR/$name.yaml" ] || { warn "跳过 $name（没有 scenarios/$name.yaml）"; continue; }

  for r in $(seq 1 "$REPEATS"); do
    tag=""
    [ "$REPEATS" -gt 1 ] && tag="_r$r"

    if [ "$REPLAY_ONLY" -eq 0 ]; then
      if [ "$FORCE_RECORD" -eq 1 ] || ! bag_exists "$DATA_DIR/${name}${tag}"; then
        record_one "$name" "$tag" || { FAILED+=("record:$name$tag"); continue; }
      else
        log "复用已有 bag: experiments/data/${name}${tag}"
        # 复用不等于免检：包的年龄可能比代码还大（比如早先版本录的），
        # 或者当初记录它的那次运行本身就有问题。这里照样验一遍。
        validate_bag "$name$tag" "$DATA_DIR/${name}${tag}" \
          || { FAILED+=("badbag:$name$tag"); continue; }
      fi
    else
      bag_exists "$DATA_DIR/${name}${tag}" || {
        warn "没有 bag：experiments/data/${name}${tag}（去掉 --replay-only 先录）"
        FAILED+=("nobag:$name$tag")
        continue
      }
      # --replay-only 也要验包：一个 6 秒的包能被当成"录好了"，
      # 回放完才发现帧数少得没法算指标 —— 那两轮回放的时间就白花了。
      validate_bag "$name$tag" "$DATA_DIR/${name}${tag}" \
        || { FAILED+=("badbag:$name$tag"); continue; }
    fi

    # --record-only：录完就停。调试时特别有用 ——
    # 录包要开 Gazebo（慢、易受图形环境影响的），回放只吃包。
    # 先用它确认"数据录对了"，再用 --replay-only 反复调算法那条链，
    # 不用每次都重新仿真。
    if [ "$RECORD_ONLY" -eq 1 ]; then
      log "  --record-only：只录包，不跑算法（之后用 --replay-only 回放）"
      continue
    fi

    for algo in "${ALGOS[@]}"; do
      if replay_one "$name" "$tag" "$algo"; then
        eval_one "$name" "$tag" || FAILED+=("eval:$name$tag:$algo")
      else
        FAILED+=("replay:$name$tag:$algo")
      fi
    done
  done
done

# ---------------------------------------------------------------- 4) 汇总
# --record-only 到此为止：只录包，**绝不能**去评测 results/raw 里上一次的旧 CSV。
# 那些文件可能来自几个小时前、甚至另一个场景/算法，混进来会得到一张
# "看起来是本次结果"的表（真机上就这样把 5 秒旧包的数据当成新数据评了一遍）。
if [ "$RECORD_ONLY" -eq 1 ]; then
  echo
  log "--record-only：跳过评测/画图/体检/汇总"
  log "已录好的包："
  for name in "${SCENARIOS[@]}"; do
    b="$DATA_DIR/$name"
    if bag_exists "$b"; then
      printf '  %-16s %8s s  %s\n' "$name" "$(bag_duration "$b")" "$(bag_files "$b")"
    fi
  done
  echo
  log "下一步：bash experiments/run_bench.sh --replay-only --scenarios ${SCENARIOS[*]}"
  write_manifest || true
  if [ ${#FAILED[@]} -gt 0 ]; then
    warn "${#FAILED[@]} 个任务失败: ${FAILED[*]}"
    exit 1
  fi
  exit 0
fi

ALL_CSV=()
while IFS= read -r f; do ALL_CSV+=("$f"); done < <(find "$RAW_DIR" -name '*.csv' -size +1k 2>/dev/null | sort)

if [ ${#ALL_CSV[@]} -gt 0 ]; then
  log "汇总 ${#ALL_CSV[@]} 条轨迹 -> results/tables/all_runs.{md,csv}"
  python3 "$EVAL" --csv "${ALL_CSV[@]}" \
    --md-out "$TABLES_DIR/all_runs.md" \
    --csv-out "$TABLES_DIR/all_runs.csv" || warn "汇总评测失败"

  if [ "$SKIP_FIGURES" -eq 0 ]; then
    log "画图 -> results/figures/"
    python3 "$FIGS" --raw-dir "$RAW_DIR" --out "$ROOT/results/figures" || \
      warn "画图失败（多半是没装 python3-matplotlib，不影响数字）"
  fi

  # 全部场景跑完，再统一体检一次（这一次的输出是写报告前的最后一道闸）
  if [ -f "$TABLES_DIR/all_runs.csv" ]; then
    log "数据可信度体检"
    python3 "$ROOT/experiments/sanity_check.py" "$TABLES_DIR/all_runs.csv" \
      --json-dir "$JSON_DIR" 2>&1 | tee "$TABLES_DIR/sanity.txt" || \
      warn "有 FAIL 项 —— 修好之前不要用这批数字写报告（见上）"
  fi

  echo
  echo "=================================================================="
  echo " 结果表: results/tables/all_runs.md"
  echo " 原始数据: results/raw/<scenario>/<algo>.csv"
  echo " JSON 明细: results/json/<scenario>.json"
  echo " 图: results/figures/"
  echo "=================================================================="
  sed 's/^/  /' "$TABLES_DIR/all_runs.md" 2>/dev/null | head -40
fi

# ---------------------------------------------------------------- 5) 清单
# 写进报告的每个数字都应该能回答"这批数据是用哪一版代码、在哪台机器上、
# 什么时候跑出来的"。manifest.json 就是那个答案，写报告时直接引用它。
write_manifest || warn "写 manifest.json 失败（不影响结果表）"

if [ ${#FAILED[@]} -gt 0 ]; then
  echo
  warn "${#FAILED[@]} 个任务失败: ${FAILED[*]}"
  exit 1
fi
log "全部完成"
