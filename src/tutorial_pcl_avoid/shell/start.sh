#!/usr/bin/env bash
# 与 raicom26_ws 一样：建立 tmux 窗口，再向交互式 shell 发送各部分启动命令。
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SESSION="${STAGE4_SESSION:-stage4_pcl}"

usage() {
  cat <<'USAGE'
用法：
  bash src/tutorial_pcl_avoid/shell/start.sh [seed:=7]       一起启动仿真、主控和监控
  bash src/tutorial_pcl_avoid/shell/start.sh --stop         停止本任务
  bash src/tutorial_pcl_avoid/shell/start.sh test [点云文件 [真值文件]]

--detach  启动后不进入 tmux，之后可执行 tmux attach -t stage4_pcl
可覆盖 sim_launch、world、cloud_topic、msg_type、cloud_frame。
USAGE
}

case "${1:-}" in -h|--help) usage; exit 0 ;; esac
case "$SESSION" in ''|*[!a-zA-Z0-9_-]*) echo '无效的 STAGE4_SESSION。' >&2; exit 1 ;; esac
if [ "${1:-}" = --stop ]; then
  if tmux has-session -t "=$SESSION" 2>/dev/null; then
    while IFS= read -r pane; do
      tmux send-keys -t "$pane" C-c
    done < <(tmux list-panes -s -t "=$SESSION" -F '#{pane_id}')
    echo '正在等待本任务的程序退出……'
    for ((attempt=0; attempt<40; ++attempt)); do
      commands="$(tmux list-panes -s -t "=$SESSION" -F '#{pane_current_command}' 2>/dev/null || true)"
      busy=false
      while IFS= read -r name; do
        case "$name" in ''|bash|zsh|sh) ;; *) busy=true ;; esac
      done <<< "$commands"
      if ! "$busy"; then
        tmux kill-session -t "=$SESSION" 2>/dev/null || true
        echo '本任务已停止。'
        exit 0
      fi
      sleep 0.5
    done
    echo "部分程序仍在退出，请查看 tmux 会话 $SESSION。" >&2
    exit 1
  else
    echo '本任务尚未启动。'
  fi
  exit 0
fi

if [ ! -f "$WS/devel/setup.bash" ]; then
  echo '请先实现回调，并在工作空间根目录执行 catkin_make。' >&2
  exit 1
fi
saved_ros_paths="${ROS_PACKAGE_PATH:-}"
source "$WS/devel/setup.bash"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$saved_ros_paths"
if [ "${1:-}" = test ]; then
  shift
  if [ "$#" -gt 2 ]; then usage >&2; exit 1; fi
  exec "$WS/devel/lib/tutorial_pcl_avoid/offline_test" "$@"
fi

command -v tmux >/dev/null || { echo '找不到 tmux，请先安装 tmux。' >&2; exit 1; }
if [ ! -x "$WS/devel/lib/tutorial_pcl_avoid/mission_node" ]; then
  echo '请先实现回调并执行 catkin_make。' >&2
  exit 1
fi

sim_args=()
sim_launch=''
cloud_topic=''
msg_type=''
cloud_frame=''
detach=false
for arg in "$@"; do
  case "$arg" in
    --detach) detach=true ;;
    seed:=*|world:=*) sim_args+=("$arg") ;;
    sim_launch:=*) sim_launch="${arg#*:=}" ;;
    cloud_topic:=*) cloud_topic="${arg#*:=}" ;;
    msg_type:=*) msg_type="${arg#*:=}" ;;
    cloud_frame:=*) cloud_frame="${arg#*:=}" ;;
    *) echo "不支持的参数：$arg" >&2; usage >&2; exit 1 ;;
  esac
done

# 学生镜像使用 simulation；本地开发镜像使用 tutorial_gazebo。
if [ -z "$sim_launch" ]; then
  if sim_dir="$(rospack find simulation 2>/dev/null)" && [ -f "$sim_dir/launch/sim.launch" ]; then
    sim_launch="$sim_dir/launch/sim.launch"
  elif sim_dir="$(rospack find tutorial_gazebo 2>/dev/null)" && [ -f "$sim_dir/launch/sim-new.launch" ]; then
    sim_launch="$WS/src/tutorial_pcl_avoid/launch/local_sim.launch"
    cloud_topic="${cloud_topic:-/livox/lidar}"
    msg_type="${msg_type:-custom}"
    cloud_frame="${cloud_frame:-body}"
  else
    echo '未找到 simulation/sim.launch 或本地 tutorial_gazebo/sim-new.launch。' >&2
    exit 1
  fi
fi
if [ ! -f "$sim_launch" ]; then echo "仿真入口不存在：$sim_launch" >&2; exit 1; fi
cloud_topic="${cloud_topic:-/cloud_registered}"
msg_type="${msg_type:-pointcloud2}"
cloud_frame="${cloud_frame:-world}"
control_args=("cloud_topic:=$cloud_topic" "msg_type:=$msg_type" "cloud_frame:=$cloud_frame")

terminal_shell="${SHELL:-/bin/zsh}"
case "${terminal_shell##*/}" in bash|zsh) ;; *) terminal_shell=/bin/bash ;; esac
suffix="${terminal_shell##*/}"
printf -v quoted_setup '%q' "$WS/devel/setup.$suffix"
ENV_CMD="_stage4_ros_paths=\"\$ROS_PACKAGE_PATH\"; source $quoted_setup; export ROS_PACKAGE_PATH=\"\$ROS_PACKAGE_PATH:\$_stage4_ros_paths\""
WAIT_MASTER='until rostopic list >/dev/null 2>&1; do sleep 1; done'
master_port="$(python3 -c 'import os; from urllib.parse import urlparse; print(urlparse(os.environ.get("ROS_MASTER_URI", "http://localhost:11311")).port or 11311)')"
printf -v CMD_SIM '%q ' roslaunch tutorial_pcl_avoid avoid.launch "sim_launch:=$sim_launch" "${sim_args[@]}" "${control_args[@]}" start_mission:=false
printf -v CMD_MAIN '%q ' roslaunch tutorial_pcl_avoid mission.launch "${control_args[@]}"
printf -v CMD_CLOUD '%q ' rostopic hz "$cloud_topic"

send_command() {
  tmux send-keys -l -t "$1" "$2"
  tmux send-keys -t "$1" C-m
}

if ! tmux has-session -t "=$SESSION" 2>/dev/null; then
  # 窗口 0：ROS 核心与仿真。
  core="$(tmux new-session -d -s "$SESSION" -n Core_Sim -c "$WS" -P -F '#{pane_id}' "$terminal_shell -i")"
  sim="$(tmux split-window -h -t "$core" -c "$WS" -P -F '#{pane_id}' "$terminal_shell -i")"
  tmux select-pane -t "$core" -T roscore
  tmux select-pane -t "$sim" -T Gazebo
  send_command "$core" "$ENV_CMD; if rostopic list >/dev/null 2>&1; then echo '使用现有 ROS master'; else roscore -p $master_port; fi"
  send_command "$sim" "$ENV_CMD; $WAIT_MASTER; $CMD_SIM"

  # 窗口 1：主控、位置、飞控状态与点云频率。
  main="$(tmux new-window -t "$SESSION:" -n Control -c "$WS" -P -F '#{pane_id}' "$terminal_shell -i")"
  pose="$(tmux split-window -h -t "$main" -c "$WS" -P -F '#{pane_id}' "$terminal_shell -i")"
  state="$(tmux split-window -v -t "$main" -c "$WS" -P -F '#{pane_id}' "$terminal_shell -i")"
  cloud="$(tmux split-window -v -t "$pose" -c "$WS" -P -F '#{pane_id}' "$terminal_shell -i")"
  tmux select-pane -t "$main" -T main_control
  tmux select-pane -t "$pose" -T position
  tmux select-pane -t "$state" -T flight_state
  tmux select-pane -t "$cloud" -T cloud_hz
  send_command "$main" "$ENV_CMD; $WAIT_MASTER; $CMD_MAIN"
  send_command "$pose" "$ENV_CMD; $WAIT_MASTER; rostopic echo /mavros/local_position/pose"
  send_command "$state" "$ENV_CMD; $WAIT_MASTER; rostopic echo /mavros/state"
  send_command "$cloud" "$ENV_CMD; $WAIT_MASTER; $CMD_CLOUD"
  tmux select-layout -t "$SESSION:Control" tiled >/dev/null
  tmux set-window-option -t "$SESSION:Control" pane-border-status top >/dev/null
  tmux set-window-option -t "$SESSION:Control" pane-border-format '#{pane_title}' >/dev/null
  tmux select-window -t "$SESSION:Control"
  tmux select-pane -t "$main"
  echo "仿真入口：$sim_launch"
  echo "点云输入：$cloud_topic ($msg_type, $cloud_frame)"
else
  echo '本任务会话已存在，连接到现有会话。'
fi

if "$detach"; then
  echo "会话：tmux attach -t $SESSION"
elif [ -n "${TMUX:-}" ]; then
  tmux switch-client -t "$SESSION"
else
  exec tmux attach-session -t "$SESSION"
fi
