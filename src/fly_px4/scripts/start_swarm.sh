#!/usr/bin/env bash
# 用法：start_swarm.sh [base1|base2|base3|route1..route7]，默认 base0。
# base1..3 对应 XTDrone/robocup 比赛地图，route1..7 为旧预设。
# 先启动仿真和裁判/行人，并使 start_positions.yaml 与仿真起点一致；脚本最后启动飞行控制。
# patrol_maps_dir 选择航线目录；--check-only 仅检查旧预设。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SETUP_FILE="$WORKSPACE_DIR/devel/setup.bash"
START_POSITIONS_FILE="$SCRIPT_DIR/../config/start_positions.yaml"

usage() {
  echo "用法：bash $0 [base1|base2|base3|route1|route2|route3|route4|route5|route6|route7]"
  echo "仿真启动：roslaunch px4 multi_vehicle_6.launch"
  echo "比赛地图启动，例如：bash $0 base2；加载 competition_base2.yaml。"
  echo "--list-routes 显示路线对照；--check-only 仅检查旧 route1..route7。"
  echo "兼容旧路线代号：base0/base4/base5/base6/base15/base16；旧 base3 请用 route2。"
  echo "六机起点统一修改：$START_POSITIONS_FILE；改完重启本脚本。"
  echo "同一文件的 patrol_maps_dir 选择 patrol_maps/patrol_maps_2；缺省使用 patrol_maps。"
}

list_routes() {
  echo "比赛地图（XTDrone/robocup）："
  echo "base1   base1.world   -> competition_base1.yaml"
  echo "base2   base2.world   -> competition_base2.yaml"
  echo "base3   base3.world   -> competition_base3.yaml"
  echo "旧 route1..route7 预设："
  echo "先把地图方向对齐：东侧是两条相距很近的南北路，图上北为上。"
  echo "代号    中/东区宽度                 北/南区高度"
  echo "route1  中区与东区近似等宽          北区与南区等高"
  echo "route2  东区明显较宽(约中区1.8倍)   北区与南区等高"
  echo "route3  中区明显较宽(约东区1.5倍)   北区与南区等高"
  echo "route4  中区与东区近似等宽          北区较矮、南区较高"
  echo "route5  中区与东区近似等宽          北区较高、南区较矮"
  echo "route6  中区明显较宽(约东区1.8倍)   北区较高、南区较矮"
  echo "route7  中区明显较宽(约东区1.8倍)   北区较矮、南区较高"
}

MAP="base0"
MAP_SET=false
CHECK_ONLY=false
for arg in "$@"; do
  case "$arg" in
    -h|--help) usage; exit 0 ;;
    --list-routes) list_routes; exit 0 ;;
    --check-only) CHECK_ONLY=true ;;
    *)
      if $MAP_SET; then
        echo "只能选择一个地图。" >&2
        usage >&2
        exit 2
      fi
      case "${arg%.world}" in
        base1) MAP=competition_base1 ;;
        base2) MAP=competition_base2 ;;
        base3) MAP=competition_base3 ;;
        route1) MAP=base0 ;;
        route2) MAP=base3 ;;
        route3) MAP=base4 ;;
        route4) MAP=base5 ;;
        route5) MAP=base6 ;;
        route6) MAP=base15 ;;
        route7) MAP=base16 ;;
        *)
          MAP="${arg%.world}"
          [[ "$MAP" == base* ]] || MAP="base$MAP"
          ;;
      esac
      MAP_SET=true
      ;;
  esac
done
case "$MAP" in
  competition_base1|competition_base2|competition_base3|base0|base3|base4|base5|base6|base15|base16) ;;
  *) echo "不支持的地图：$MAP" >&2; usage >&2; exit 2 ;;
esac

# 在启动任何 ROS 进程前校验起点与目录；目录只接受两套预设名。
PATROL_MAPS_DIR="$(python3 "$SCRIPT_DIR/start_positions.py" "$START_POSITIONS_FILE" --print-maps-dir)"
ROUTES_DIR="$SCRIPT_DIR/../config/$PATROL_MAPS_DIR"
ROUTES_FILE="$ROUTES_DIR/$MAP.yaml"
if [[ ! -f "$ROUTES_FILE" ]]; then
  echo "找不到所选巡航航线：$ROUTES_FILE" >&2
  exit 2
fi

if $CHECK_ONLY; then
  if [[ "$MAP" == competition_* ]]; then
    echo "--check-only 仅支持旧 route1..route7；比赛 base1/base2/base3 请直接启动。" >&2
    exit 2
  fi
  exec python3 "$SCRIPT_DIR/check_patrol_maps.py" --map "$MAP" --start-positions "$START_POSITIONS_FILE" --routes-dir "$ROUTES_DIR"
fi

# 在任何飞控/桥接启动前检查完整六机配置；不查询 Gazebo 或地图文件。
python3 "$SCRIPT_DIR/start_positions.py" "$START_POSITIONS_FILE"

if [[ ! -f "$SETUP_FILE" ]]; then
  echo "找不到 ROS 工作空间环境：$SETUP_FILE" >&2
  exit 1
fi

# 不加载 .bashrc，避免额外叠加 racer_ws 的同名包。
source "$SETUP_FILE"
set -u

if ! command -v roslaunch >/dev/null 2>&1; then
  echo "找不到 roslaunch，请检查 ROS 环境。" >&2
  exit 1
fi

# 比赛仅使用操作者选定的预设；启动流程不读取 world 文件、地图名或 Gazebo 真值。
echo "人工选择的巡航预设：$PATROL_MAPS_DIR/$MAP.yaml；六机坐标偏移使用上述起点配置。"

PIDS=()

cleanup() {
  trap - EXIT INT TERM
  echo "正在关闭本脚本启动的 ROS 进程……"
  if (( ${#PIDS[@]} > 0 )); then
    kill -INT "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_launch() {
  local launch_file="$1"
  shift
  local pid
  # 发现前面的 roslaunch 已退出时，不继续启动飞行控制。
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "前面的 roslaunch 已退出，停止启动。" >&2
      exit 1
    fi
  done
  echo "启动：roslaunch fly_px4 $launch_file $*"
  roslaunch fly_px4 "$launch_file" "$@" &
  PIDS+=("$!")
}

start_launch swarm_relay_ego.launch "start_positions_file:=$START_POSITIONS_FILE"
start_launch swarm_track.launch
start_launch swarm_fly_px4_1.launch "map:=$MAP" "routes_file:=$ROUTES_FILE" "start_positions_file:=$START_POSITIONS_FILE"

echo "三个启动命令已执行，按 Ctrl+C 统一停止。"
# 任一 roslaunch 结束时，退出脚本并清理其余进程。
wait -n "${PIDS[@]}"
