#!/usr/bin/env bash
# com_launch/scripts/run_node.sh
# PM2가 각 노드를 띄울 때 공통으로 거치는 wrapper.
# ROS 2 환경 + TurtleBot3 환경변수를 source한 뒤, 인자로 받은 ros2 명령을 그대로 실행한다.
#
# 사용법: run_node.sh <ros2 명령어...>
#   예) run_node.sh launch turtlebot3_bringup robot.launch.py
#       run_node.sh run com_service service_manager_node
set -e

# ── 시계 동기 대기 (2026-08-19 추가) ───────────────────────────────────
# 라즈베리파이에는 RTC가 없어서(timedatectl -> "RTC time: n/a") 부팅 시
# systemd-timesyncd가 "마지막으로 기록된 시각"으로 시계를 복원한 뒤, NTP가 붙으면
# 실제 시각으로 크게 점프한다. 껐던 시간만큼 점프폭이 커진다(실측: 하룻밤 껐다 켜니
# +15.1시간, 30분 껐다 켜니 +40분).
#
# PM2가 부팅과 동시에 노드를 띄우므로 점프 "전"에 ROS 노드가 뜨면, 이후 TF 메시지
# 스탬프가 전부 과거가 되어 tf2 버퍼(기본 10초)가 통째로 버린다. 그 결과 map->odom,
# odom->base_footprint TF가 사라지고 AMCL/Nav2가 먹통이 된다(매 부팅 100% 재현,
# 수동으로 `pm2 restart all`을 해야만 복구됐다).
#
# 네트워크가 없는 환경에서도 로봇은 동작해야 하므로(SR-023) 무한 대기하지 않고
# 최대 60초만 기다린 뒤 진행한다.
NTP_WAIT_MAX_SEC=60
for _i in $(seq 1 "$NTP_WAIT_MAX_SEC"); do
    if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; then
        break
    fi
    sleep 1
done
if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" != "yes" ]; then
    echo "[run_node.sh] 경고: ${NTP_WAIT_MAX_SEC}초 안에 시계 동기가 안 됨 - 그대로 진행합니다." >&2
    echo "[run_node.sh] 나중에 NTP가 붙어 시계가 점프하면 TF가 끊길 수 있습니다." >&2
    echo "[run_node.sh] 그때는 'pm2 restart all'로 복구하세요." >&2
fi

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-03
export ROS_DOMAIN_ID=30

source /opt/ros/humble/setup.bash
source "$HOME/com_ws/install/setup.bash"

exec ros2 "$@"
