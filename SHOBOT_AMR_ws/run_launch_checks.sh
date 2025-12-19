#!/usr/bin/env bash
set -euo pipefail
LOG="/home/shahbaz/Business/AMR/SHOBOT_AMR_ws/launch_checks.log"
: > "$LOG"
while read -r pkg file; do
  [[ -z "$pkg" ]] && continue
  echo "=== ${pkg}/${file} ===" | tee -a "$LOG"
  EXTRA=""
  if [[ "$pkg" == "shobot_bringup" ]]; then
    EXTRA="use_nav_server:=false"
  elif [[ "$pkg" == "shobot_navigation_server" ]]; then
    EXTRA="bringup_nav2:=false"
  fi
  set +e
  timeout 10s bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /home/shahbaz/Business/AMR/SHOBOT_AMR_ws/install/setup.bash
    export ROS_HOME=/home/shahbaz/Business/AMR/SHOBOT_AMR_ws/.ros
    export ROS_LOG_DIR=\$ROS_HOME/log
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 launch ${pkg} ${file} ${EXTRA}
  " >>"$LOG" 2>&1
  status=$?
  set -e
  echo "exit_code=$status" | tee -a "$LOG"
done < /home/shahbaz/Business/AMR/SHOBOT_AMR_ws/launch_check_list.txt
