#!/bin/bash
# Bunker 고도 맵핑 준비 스크립트
# tmux 3분할 후 source 완료, 실행 명령어는 입력만 해놓고 대기

WS_DIR="$HOME/ros2_ws/bunker_sim_v2_ws"
BAG_DIR="${1:-$WS_DIR/rosbag/5}"
RVIZ_CONFIG="$WS_DIR/rviz_config/elevation_mapping.rviz"
SESSION="bunker_mapping"

# 기존 세션 있으면 종료
tmux kill-session -t "$SESSION" 2>/dev/null

# 세션 생성 (pane 0 — 매핑 시스템)
tmux new-session -d -s "$SESSION"

# 3분할: 위 1칸 / 아래 2칸
tmux split-window -v -t "$SESSION"
tmux split-window -h -t "$SESSION:0.1"

# pane 0 (상단): 매핑 시스템
tmux send-keys -t "$SESSION:0.0" "conda deactivate; source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash" Enter
tmux send-keys -t "$SESSION:0.0" "ros2 launch bunker_util system_start.launch.py"   # Enter 없음

# pane 1 (하단 좌): bag 재생
tmux send-keys -t "$SESSION:0.1" "conda deactivate; source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t "$SESSION:0.1" "ros2 bag play $BAG_DIR --clock"   # Enter 없음

# pane 2 (하단 우): RViz
tmux send-keys -t "$SESSION:0.2" "conda deactivate; source /opt/ros/humble/setup.bash" Enter
tmux send-keys -t "$SESSION:0.2" "rviz2 -d $RVIZ_CONFIG"   # Enter 없음

# 포커스를 pane 0으로
tmux select-pane -t "$SESSION:0.0"
tmux attach-session -t "$SESSION"
