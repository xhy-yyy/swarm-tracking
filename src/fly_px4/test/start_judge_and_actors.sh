#!/usr/bin/env bash
# 用法：start_judge_and_actors.sh actors|judge|both|stop。
# 使用随本项目提供的本地比赛脚本快照。
# both 先启动行人再启动裁判；比赛计时从裁判启动开始。
set -u

ROBOCUP_DIR="$(rospack find swarm_simulation)/scripts/competition"
PY=python3
UAV_TYPE=typhoon_h480
ACTOR_NUM=6

if [ ! -d "$ROBOCUP_DIR" ]; then
  echo "找不到 $ROBOCUP_DIR" >&2
  exit 1
fi

start_actors() {
  cd "$ROBOCUP_DIR" || exit 1
  for i in $(seq 0 $((ACTOR_NUM - 1))); do
    "$PY" control_actor.py "$i" &
    echo "  行人 actor_$i 已启动 (pid $!)"
  done
}

start_judge() {
  cd "$ROBOCUP_DIR" || exit 1
  echo "  裁判启动 —— 600 秒计时从现在开始"
  "$PY" score_cal.py "$UAV_TYPE"
}

case "${1:-both}" in
  actors)
    start_actors
    echo "行人已全部启动, Ctrl+C 或 'stop' 停止"
    wait
    ;;
  judge)
    start_judge
    ;;
  both)
    start_actors
    sleep 2
    start_judge          # 前台运行, 便于直接看 find/OK 打印
    ;;
  stop)
    pkill -f 'control_actor\.py' && echo "行人已停止"
    pkill -f 'score_cal\.py'     && echo "裁判已停止"
    ;;
  *)
    echo "用法: $0 {actors|judge|both|stop}" >&2
    exit 1
    ;;
esac
