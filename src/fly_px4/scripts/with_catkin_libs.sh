#!/usr/bin/env bash
# 作为 roslaunch 的 launch-prefix，过滤 racer_ws 的同名库和包搜索路径。
# 保留其余依赖搜索顺序；工作空间路径可用 PURGE_WS 覆盖。
set -u

# 要从 LD_LIBRARY_PATH 里摘掉的工作空间 (可用 ~PURGE_WS 覆盖, 默认 racer_ws)
PURGE_WS="${PURGE_WS:-$HOME/racer_ws}"

if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  NEW_PATH=""
  DROPPED=""
  # 按 : 切开逐段过滤, 保持原有顺序
  IFS=':' read -ra PARTS <<< "$LD_LIBRARY_PATH"
  for p in "${PARTS[@]}"; do
    [ -z "$p" ] && continue
    case "$p" in
      "$PURGE_WS"/*|"$PURGE_WS")
        DROPPED="${DROPPED:+$DROPPED, }$p"
        ;;
      *)
        NEW_PATH="${NEW_PATH:+$NEW_PATH:}$p"
        ;;
    esac
  done
  export LD_LIBRARY_PATH="$NEW_PATH"
  if [ -n "$DROPPED" ]; then
    echo "[with_catkin_libs] 已从 LD_LIBRARY_PATH 摘掉: $DROPPED" >&2
  fi
fi

# 同理摘掉 ROS_PACKAGE_PATH 里的该工作空间, 免得 roslaunch/rospack 解析到那边的同名包
if [ -n "${ROS_PACKAGE_PATH:-}" ]; then
  NEW_RPP=""
  IFS=':' read -ra RPARTS <<< "$ROS_PACKAGE_PATH"
  for p in "${RPARTS[@]}"; do
    [ -z "$p" ] && continue
    case "$p" in
      "$PURGE_WS"/*|"$PURGE_WS") ;;
      *) NEW_RPP="${NEW_RPP:+$NEW_RPP:}$p" ;;
    esac
  done
  export ROS_PACKAGE_PATH="$NEW_RPP"
fi

# exec 掉自己: 这样 roslaunch 的进程监控/信号(SIGINT/SIGTERM)直接作用在真正的节点上,
# 中间不留一层 shell —— 否则 Ctrl+C 杀不干净, respawn 也会认错进程。
exec "$@"
