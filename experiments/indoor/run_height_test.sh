#!/usr/bin/env bash
# LiDAR 높이 한 개로 실내 케이지 시험 비행을 처음부터 끝까지 돌린다.
#   월드 생성(LIDAR_H) → PX4+Gazebo → 통신 에이전트·브리지 → 트래커·점 측정기
#   → indoor_survey_mission 비행 → 정리
#
# 사용:  run_height_test.sh <높이 m> [결과 폴더]
#   예)  for h in 0.6 0.8 1.0 1.2 1.5 1.8; do ./run_height_test.sh $h; done
#   HEADLESS=1 을 주면 Gazebo 화면 없이 돈다.
#   MODEL=x500_cf2x2 로 기체 모델을 바꾼다 (기본 x500_cf2 = Crazyflie 실물 크기).
#   LAYOUT=short 로 보관 중인 짧은 변 배치를 쓴다 (기본 long = 긴 변 가운데).
#   결과 폴더 이름에는 항상 배치가 붙는다 (height_sweep[_기체]_long 등).
#   RUN_NAME=F1 이면 결과 하위 폴더 이름이 h120 대신 F1 이 된다 (반복 비행용).
#   RECORD_BAG=1 이면 LiDAR·참값·PX4 위치를 <결과>/bag 에 녹화한다
#   (compare_fusion.sh 로 PF/MF/SU를 같은 입력에 재생해 비교할 때 필요, 1분에 약 2 GB).
#   트래커·측정기는 생성된 world SDF에서 LiDAR 위치를 읽는다 (yaml 값은 쓰지 않음).
#
# 결과: <결과 폴더>/h080/ 에 points.csv(프레임별 점 수), tracker.log, px4.log,
#       preflight.txt(PX4 비행 전 점검), mission.log
set -eo pipefail

H=${1:?사용: run_height_test.sh <높이 m> [결과 폴더]}
ROOT=$HOME/drone_project
DPS=$ROOT/DronePositioningSystem
PX4=$ROOT/PX4-Autopilot
TAG=h$(printf %03d "$(python3 -c "print(round($H*100))")")
LAYOUT=${LAYOUT:-long}
WORLD=indoor_cage_$TAG
[ "$LAYOUT" = long ] || WORLD=indoor_cage_${LAYOUT}_$TAG
MODEL=${MODEL:-x500_cf2}
DEFAULT_OUT=$ROOT/flight_logs/indoor/height_sweep
[ "$MODEL" = x500_cf2 ] || DEFAULT_OUT=${DEFAULT_OUT}_${MODEL#x500_}
DEFAULT_OUT=${DEFAULT_OUT}_$LAYOUT
OUT=${2:-$DEFAULT_OUT}/${RUN_NAME:-$TAG}     # RUN_NAME=F1 처럼 주면 폴더 이름을 바꾼다
PARAMS=$DPS/ros2_ws/src/drone_localization/params/lidar_localization_indoor.yaml

# dps-kill 직후에는 프로세스가 아직 내려가는 중일 수 있어 잠시 기다린다
sim_running() { pgrep -f 'build/px4_sitl_default/bin/px4' >/dev/null || pgrep -f 'gz sim' >/dev/null; }
for _ in $(seq 15); do sim_running || break; sleep 1; done
if sim_running; then
    echo "PX4나 Gazebo가 아직 떠 있습니다. dps-kill 후 다시 실행하세요." >&2
    exit 1
fi
# 지상국(QGC) 연결이 없으면 PX4가 시동을 거부한다 (gcs_connection_lost)
if ! pgrep -f QGroundControl >/dev/null; then
    echo "QGroundControl이 꺼져 있습니다. 먼저 켜 두세요: ~/Applications/QGroundControl.AppImage" >&2
    exit 1
fi
mkdir -p "$OUT"
echo "=== LiDAR 높이 $H m, 배치 $LAYOUT ($WORLD), 기체 $MODEL → $OUT"

# 1) 월드 생성·배포
LIDAR_H=$H LIDAR_LAYOUT=$LAYOUT python3 "$DPS/gazebo/worlds/generate_indoor_cage.py" >/dev/null
cp "$DPS/gazebo/worlds/$WORLD.sdf" "$DPS/gazebo/worlds/${WORLD}_coordinates.txt" \
   "$PX4/Tools/simulation/gz/worlds/"

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
source "$DPS/ros2_ws/install/setup.bash"
SDF=$DPS/gazebo/worlds/$WORLD.sdf
COUNTER=$DPS/experiments/indoor/count_target_points.py
read -ra LIDAR_ARGS <<<"$(python3 "$COUNTER" --world-sdf "$SDF" --params "$PARAMS" --out /dev/null --print-tracker-args)"
echo "LiDAR 위치(world SDF): ${LIDAR_ARGS[*]}"

PGIDS=()
start() {   # start <로그파일> <명령...> — 새 프로세스 그룹으로 띄워 정리 때 통째로 끈다
    local log=$1; shift
    setsid "$@" >"$log" 2>&1 &
    PGIDS+=($!)
}
cleanup() {
    echo "--- 정리"
    for g in "${PGIDS[@]}"; do kill -INT -- "-$g" 2>/dev/null || true; done
    sleep 2
    for g in "${PGIDS[@]}"; do kill -KILL -- "-$g" 2>/dev/null || true; done
    pkill -f "gz sim.*$WORLD" 2>/dev/null || true
}
trap cleanup EXIT

# 2) PX4 + Gazebo (데몬 모드: pxh 쉘 없이)
start "$OUT/px4.log" env -C "$PX4/build/px4_sitl_default/rootfs" \
    PX4_SIM_MODEL="gz_$MODEL" PX4_GZ_WORLD="$WORLD" PX4_GZ_NO_FOLLOW=1 GZ_IP=127.0.0.1 \
    ../bin/px4 -d
echo "PX4 시작 대기..."
for _ in $(seq 90); do
    grep -q "Ready for takeoff" "$OUT/px4.log" && break
    sleep 1
done
grep -q "Ready for takeoff" "$OUT/px4.log" || { echo "PX4가 90초 안에 준비되지 않음 (px4.log 확인)" >&2; exit 1; }

# 3) 통신·브리지·트래커·측정기
start "$OUT/agent.log" MicroXRCEAgent udp4 -p 8888
start "$OUT/bridge.log" ros2 run ros_gz_bridge parameter_bridge \
    /lidar1/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
    /lidar2/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
    "/world/$WORLD/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V"
start "$OUT/tracker.log" ros2 run drone_localization lidar_drone_tracker_cov --ros-args \
    -r __node:=lidar_drone_tracker --params-file "$PARAMS" "${LIDAR_ARGS[@]}"
start "$OUT/counter.log" python3 "$COUNTER" \
    --out "$OUT/points.csv" --params "$PARAMS" --world "$WORLD" --world-sdf "$SDF"
cp "$SDF" "$OUT/world.sdf"      # 재생할 때 LiDAR 위치를 다시 읽는 원본
if [ -n "${RECORD_BAG:-}" ]; then
    rm -rf "$OUT/bag"
    start "$OUT/bag.log" ros2 bag record -o "$OUT/bag" \
        /lidar1/points /lidar2/points "/world/$WORLD/dynamic_pose/info" \
        /fmu/out/vehicle_local_position_v1
fi
sleep 8

# 비행 전 점검 결과를 남긴다 (시동이 거부될 때 원인 확인용)
(cd "$PX4/build/px4_sitl_default/rootfs" && ../bin/px4-commander check) >"$OUT/preflight.txt" 2>&1 || true

# 4) 비행 (착륙·disarm 하면 스스로 끝남)
echo "비행 시작..."
# MISSION 으로 임무와 인자를 바꾼다 (예: MISSION="indoor_shapes_mission helix")
if timeout 180 ros2 run drone_mission ${MISSION:-indoor_survey_mission} >"$OUT/mission.log" 2>&1; then
    echo "비행 완료"
else
    echo "비행이 180초 안에 끝나지 않음 — mission.log, preflight.txt 확인" >&2
fi
sleep 3
echo "기록: $(($(wc -l <"$OUT/points.csv") - 1))행 → $OUT/points.csv"
