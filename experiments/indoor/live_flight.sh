#!/usr/bin/env bash
# 실내 케이지 라이브 비행 한 번 — 보면서 날리고, 끝나면 경로 그림이 자동으로 뜬다.
#
#   PX4+Gazebo(화면) → 통신 에이전트 → 트래커(launch+실내 yaml) → 참값 브리지
#   → 경로 표시(path_viz) → RViz → [RECORD_BAG=1 이면 bag 녹화] → 비행 미션
#   → 끝나면: 경로 그림(path_result.png) 열기 → [bag 이 있으면 PF/MF/SU 재생 비교(fusion_compare.png)]
#   → Enter 를 누르면 시뮬레이션까지 모두 정리
#
# 사용:  live_flight.sh [이름]        (기본 이름 run_월일_시분)
#   결과: ~/drone_project/flight_logs/indoor/live/<이름>/  (true_path.csv, estimated_path.csv, path_result.png, 로그)
#   환경변수: MODEL(기본 x500_cf2x2), WORLD(기본 indoor_cage), MISSION(기본 indoor_survey_mission,
#             예: "indoor_shapes_mission helix"), RECORD_BAG=1(원본 점군 녹화 + 재생 비교, 1분에 2~3 GB),
#             NO_COMPARE=1(녹화는 하되 재생 비교 생략)
#   QGroundControl이 켜져 있어야 시동이 걸린다.
set -eo pipefail

NAME=${1:-run_$(date +%m%d_%H%M)}
ROOT=$HOME/drone_project
DPS=$ROOT/DronePositioningSystem
PX4=$ROOT/PX4-Autopilot
HERE=$DPS/experiments/indoor
OUT=$ROOT/flight_logs/indoor/live/$NAME
MODEL=${MODEL:-x500_cf2x2}
WORLD=${WORLD:-indoor_cage}

sim_running() { pgrep -f 'build/px4_sitl_default/bin/px4' >/dev/null || pgrep -f 'gz sim' >/dev/null; }
for _ in $(seq 15); do sim_running || break; sleep 1; done
if sim_running; then echo "PX4나 Gazebo가 아직 떠 있습니다. dps-kill 후 다시 실행하세요." >&2; exit 1; fi
if ! pgrep -f QGroundControl >/dev/null; then
    echo "QGroundControl이 꺼져 있습니다. 먼저 켜 두세요: ~/Applications/QGroundControl.AppImage &" >&2; exit 1
fi
if [ -e "$OUT" ]; then echo "$OUT 가 이미 있습니다. 다른 이름을 주세요." >&2; exit 1; fi
mkdir -p "$OUT"
cp "$DPS/gazebo/worlds/$WORLD.sdf" "$OUT/world.sdf"
echo "=== 라이브 비행 $NAME (월드 $WORLD, 기체 $MODEL) → $OUT"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
source "$DPS/ros2_ws/install/setup.bash"

PGIDS=()
start() {   # start <로그파일> <명령...> — 새 프로세스 그룹으로 띄운다
    local log=$1; shift
    setsid "$@" >"$log" 2>&1 &
    PGIDS+=($!)
    LAST=$!
}
stop_group() {   # stop_group <pgid> — Ctrl+C 처럼 끄고 끝날 때까지 기다린다
    kill -INT -- "-$1" 2>/dev/null || true
    for _ in $(seq 20); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done
    kill -KILL -- "-$1" 2>/dev/null || true
}
cleanup() {
    echo "--- 정리"
    for g in "${PGIDS[@]}"; do kill -INT -- "-$g" 2>/dev/null || true; done
    sleep 2
    for g in "${PGIDS[@]}"; do kill -KILL -- "-$g" 2>/dev/null || true; done
    pkill -f "gz sim.*$WORLD" 2>/dev/null || true
}
trap cleanup EXIT

# 1) PX4 + Gazebo (화면 포함, pxh 쉘 없이)
start "$OUT/px4.log" env -C "$PX4/build/px4_sitl_default/rootfs" \
    PX4_SIM_MODEL="gz_$MODEL" PX4_GZ_WORLD="$WORLD" PX4_GZ_NO_FOLLOW=1 GZ_IP=127.0.0.1 \
    ../bin/px4 -d
echo "PX4·Gazebo 시작 대기..."
for _ in $(seq 90); do grep -q "Ready for takeoff" "$OUT/px4.log" && break; sleep 1; done
grep -q "Ready for takeoff" "$OUT/px4.log" || { echo "PX4가 90초 안에 준비되지 않음 (px4.log 확인)" >&2; exit 1; }

# 2) 통신·트래커·참값·경로 표시·RViz·녹화
start "$OUT/agent.log" MicroXRCEAgent udp4 -p 8888
start "$OUT/tracker.log" ros2 launch drone_localization lidar_localization.launch.py \
    params_file:=lidar_localization_indoor.yaml
start "$OUT/gt_bridge.log" ros2 run ros_gz_bridge parameter_bridge \
    "/world/$WORLD/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V"
start "$OUT/path_viz.log" python3 "$HERE/path_viz.py" --world "$WORLD" --save "$OUT"
VIZ=$LAST
start "$OUT/rviz.log" rviz2 -d "$HERE/indoor.rviz"
BAG=
if [ -n "${RECORD_BAG:-}" ]; then
    start "$OUT/bag.log" ros2 bag record -o "$OUT/bag" \
        /lidar1/points /lidar2/points "/world/$WORLD/dynamic_pose/info" /fmu/out/vehicle_local_position_v1
    BAG=$LAST
fi
echo "RViz${BAG:+·녹화} 준비 대기..."
sleep 10

# 3) 비행 (착륙·disarm 하면 스스로 끝남)
echo "비행 시작 — Gazebo·RViz 창에서 보세요"
if timeout 180 ros2 run drone_mission ${MISSION:-indoor_survey_mission} >"$OUT/mission.log" 2>&1; then
    echo "비행 완료"
else
    echo "비행이 180초 안에 끝나지 않음 — $OUT/mission.log 확인" >&2
fi
sleep 2

# 4) 녹화·경로 저장 → 경로 그림 열기
[ -n "$BAG" ] && stop_group "$BAG"
stop_group "$VIZ"
grep -E "저장|건너뜀" "$OUT/path_viz.log" || true
[ -f "$OUT/path_result.png" ] && xdg-open "$OUT/path_result.png" >/dev/null 2>&1 &

# 5) PF/MF/SU 재생 비교 → 그림 열기
if [ -z "${NO_COMPARE:-}" ] && [ -d "$OUT/bag" ]; then
    echo "PF/MF/SU 재생 비교 중 (2~3분)..."
    "$HERE/compare_fusion.sh" "$OUT" >"$OUT/compare.log" 2>&1 && \
        python3 "$HERE/plot_fusion.py" "$OUT/fusion" 2>/dev/null | grep -v -E "Axes3D|warn" && \
        xdg-open "$OUT/fusion/fusion_compare.png" >/dev/null 2>&1 &
    wait $! || echo "재생 비교 실패 — $OUT/compare.log 확인" >&2
fi

echo "결과: $OUT"
read -rp "Enter 를 누르면 Gazebo·RViz까지 모두 닫습니다... " _
