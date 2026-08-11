#!/usr/bin/env bash
# com_launch/scripts/run_node.sh
# PM2가 각 노드를 띄울 때 공통으로 거치는 wrapper.
# ROS 2 환경 + TurtleBot3 환경변수를 source한 뒤, 인자로 받은 ros2 명령을 그대로 실행한다.
#
# 사용법: run_node.sh <ros2 명령어...>
#   예) run_node.sh launch turtlebot3_bringup robot.launch.py
#       run_node.sh run com_service service_manager_node
set -e

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-03
export ROS_DOMAIN_ID=30

source /opt/ros/humble/setup.bash
source "$HOME/com_ws/install/setup.bash"

exec ros2 "$@"
