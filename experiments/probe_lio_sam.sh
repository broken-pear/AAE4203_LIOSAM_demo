#!/usr/bin/env bash
# =============================================================================
# probe_lio_sam.sh —— 把"LIO-SAM 跑不起来"拆成两个互斥的可能
#
#   可能 A：LIO-SAM 在这台机器上根本出不了位姿（环境/依赖/参数）
#   可能 B：LIO-SAM 没问题，问题在"回放"这条链（QoS / 时钟 / 同步）
#
# 做法：**不开 bag**，直接实时仿真（Demo 2 用过的 launch），跑 45 秒，
# 看 /lio_sam/mapping/odometry 到底有没有数据。
#
#   A 有数据  -> 环境没问题，问题在回放链路（去看 replay 日志 + watchdog 表）
#   A 没数据  -> LIO-SAM 本身在这台机器上就不工作（看四段里程碑卡在哪）
#
# 用法：bash experiments/probe_lio_sam.sh
# =============================================================================
set -uo pipefail

LOG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/results/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/probe_lio_sam.log"

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
info() { echo -e "${GREEN}==>${NC} $*"; }
warn() { echo -e "${YELLOW}[warn]${NC} $*"; }
bad()  { echo -e "${RED}[FAIL]${NC} $*"; }

command -v ros2 >/dev/null || { bad "找不到 ros2（先 source /opt/ros/jazzy/setup.bash）"; exit 1; }

info "1/4 启动实时仿真（LIO-SAM 四节点 + Gazebo，无 RViz，45 秒）"
info "    日志: $LOG"
timeout -s INT --kill-after=15 45 ros2 launch lio_sam_gz_bringup lio_sam_gazebo.launch.py \
  rviz:=false drive:=auto use_sim_time:=true > "$LOG" 2>&1 &
LAUNCH_PID=$!

info "2/4 等 20 秒让 Gazebo 起来、LIO-SAM 初始化"
sleep 20

info "3/4 现场探测话题（不看日志，直接问 ROS 图）"
echo "    --- 相关话题 ---"
ros2 topic list 2>/dev/null | grep -E '^/(points|points_raw|imu/data|imu_raw|gt/odom|lio_sam)' \
  | sed 's/^/      /' || echo "      （一个都没有 —— Gazebo 或桥接没起来）"

echo "    --- 发布者数量 ---"
for t in /points /imu/data /lio_sam/deskew/cloud_info /lio_sam/feature/cloud_info \
         /lio_sam/mapping/odometry; do
  n=$(ros2 topic info "$t" 2>/dev/null | awk '/Publisher count/ {print $3}')
  printf '      %-34s publishers=%s\n' "$t" "${n:-?}"
done

echo "    --- 关键话题有没有数据（各看 5 秒）---"
for t in /points /lio_sam/deskew/cloud_info /lio_sam/feature/cloud_info /lio_sam/mapping/odometry; do
  hz=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')
  if [ -n "${hz:-}" ]; then
    echo -e "      ${GREEN}OK${NC}   $t  ≈ ${hz} Hz"
  else
    echo -e "      ${RED}无数据${NC} $t"
  fi
done

info "4/4 收尾并读里程碑"
kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
pkill -INT -f 'lio_sam_' 2>/dev/null || true
pkill -INT -f gz-sim 2>/dev/null || true
sleep 2

echo
echo "=== LIO-SAM 四段流水线（日志里的启动横幅）==="
for ms in 'IMU Preintegration' 'Image Projection Started' \
          'Feature Extraction Started' 'Map Optimization Started'; do
  if grep -qi "$ms" "$LOG" 2>/dev/null; then
    echo -e "  ${GREEN}[OK  ]${NC} $ms"
  else
    echo -e "  ${RED}[MISS]${NC} $ms"
  fi
done

echo
echo "=== 日志里的错误行 ==="
grep -inE 'error|fail|not enough|terminate|exception|shutdown|not in dense' "$LOG" \
  | head -12 | sed 's/^/  /' || echo "  （没有）"

echo
echo "=== 结论怎么读 ==="
cat <<'EOF'
  /lio_sam/mapping/odometry 有 Hz   -> LIO-SAM 本身没问题，问题在"回放"这条链。
                                       下一步查 replay 日志 + QoS（bag play 的发布端）。
  deskew/cloud_info 有、feature 没有 -> 卡在特征提取（看 featureExtraction 的日志）
  全部没数据、四段都 OK              -> imageProjection 在"Waiting for IMU data ..."
                                       循环（点云/IMU 时间戳对不上）
  四段有 MISS                        -> 节点没起来，看上面第一条 error
EOF
