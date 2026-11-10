#!/usr/bin/env bash
# 실내 경로 5종 × N회 시뮬: 실기 배치(LIDAR_LAYOUT=real)에서 비행 → PF/MF/SU 재생 → 평가
#
# 사용:  shapes_replicate.sh [단계...]     (기본: record replay eval)
#   환경변수: ROOT(기본 ~/drone_project/flight_logs/indoor/shapes), N(기본 3), SHAPES, MODEL(기본 x500_cf2x2),
#             CLOCK(기본 arrival = 수신 시각, 라이브·실기와 같음 | stamp = 센서 측정 시각)
#   QGroundControl 이 켜져 있어야 한다 (record 단계).
#
# 재생 시계: arrival 은 bag 수신 시각(라이브와 같다), stamp 는 센서 측정 시각. Gazebo 는 센서를 순서대로
# 처리해 같은 시각의 두 스캔이 수십 ms 차이로 도착한다 (실기는 위상 고정이라 같은 시각에 도착).
# record 는 재생 비교를 위해 항상 bag 을 녹화한다 (비행 1회 2~3 GB). ROI 는 실기와 같은 4 x 3 m.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
DPS=$HOME/drone_project/DronePositioningSystem
ROOT=${ROOT:-$HOME/drone_project/flight_logs/indoor/shapes}
N=${N:-3}
SHAPES=${SHAPES:-helix circle figure8 square backforth}
MODEL=${MODEL:-x500_cf2x2}
PARAMS=$DPS/ros2_ws/src/drone_localization/params/lidar_localization_indoor.yaml
ROI=(-p roi_x_min:=-0.5 -p roi_x_max:=3.5 -p roi_y_min:=-0.5 -p roi_y_max:=2.5 -p roi_z_min:=0.10 -p roi_z_max:=2.5)
CLOCK=${CLOCK:-arrival}
STAGES=${*:-record replay eval}

record() {
    for s in $SHAPES; do
        for i in $(seq "$N"); do
            f=F_${s}_$i
            if [ -d "$ROOT/flights/$f/bag" ]; then echo "$f 녹화 있음 — 건너뜀"; continue; fi
            RUN_NAME=$f RECORD_BAG=1 MODEL=$MODEL HEADLESS=1 LAYOUT=real MISSION="indoor_shapes_mission $s" \
                "$HERE/run_height_test.sh" 1.26 "$ROOT/flights"
            sleep 3
        done
    done
}

replay_one() {   # replay_one <비행 폴더> <PF|MF|SU>
    local F=$1 M=$2 f world out L X
    f=$(basename "$F")
    world=$(python3 -c "import xml.etree.ElementTree as E; print(E.parse('$F/world.sdf').getroot().find('world').get('name'))")
    read -ra L <<<"$(python3 - "$F/world.sdf" <<'PY'
import sys, xml.etree.ElementTree as ET
p = []
for m in ET.parse(sys.argv[1]).getroot().iter('model'):
    n = m.get('name', '')
    if n.startswith('ground_lidar_'):
        i = n.rsplit('_', 1)[1]
        for k, v in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), map(float, m.find('pose').text.split())):
            p.append(f'-p lidar{i}_{k}:={v}')
print(' '.join(p))
PY
)"
    case $M in
        PF) X=(-p fusion_mode:=cloud -p process_on_arrival:=true) ;;
        MF) X=(-p fusion_mode:=position -p process_on_arrival:=true) ;;
        SU) X=(-p fusion_mode:=sequential) ;;
    esac
    out=$ROOT/cmp_state/$f/${M}0
    [ -f "$out/state_details.csv" ] && return 0
    rm -rf "$out"; mkdir -p "$out"
    echo "재생 $f $M"
    python3 -m drone_localization.replay_bag --bag "$F/bag" --out "$out" --tracker cov --clock "$CLOCK" \
        --ground-truth-topic "/world/$world/dynamic_pose/info" --require-ground-truth --quiet \
        --ros-args --params-file "$PARAMS" "${L[@]}" "${ROI[@]}" "${X[@]}" >"$out/replay.log" 2>&1
}

replay() {
    source /opt/ros/humble/setup.bash
    source "$HOME/drone_project/ros2_ws/install/setup.bash"
    source "$DPS/ros2_ws/install/setup.bash"
    for F in "$ROOT"/flights/F_*; do
        [ -d "$F/bag" ] || continue
        for M in PF MF SU; do replay_one "$F" "$M" & done
        wait
    done
}

for s in $STAGES; do
    case $s in
        record) record ;;
        replay) replay ;;
        eval)   python3 "$HERE/shapes_eval.py" "$ROOT" ;;
        *) echo "알 수 없는 단계: $s (record|replay|eval)" >&2; exit 1 ;;
    esac
done
