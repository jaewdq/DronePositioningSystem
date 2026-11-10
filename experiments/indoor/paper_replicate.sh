#!/usr/bin/env bash
# 논문(24x15, flight_logs/paper_1024) 데이터 생성 절차를 실내 케이지에서 그대로 반복한다.
#
#   1) record  같은 경로(indoor_survey_mission)를 N번 비행하며 bag 녹화  → <root>/flights/F1..FN
#   2) replay  비행마다 PF/MF/SU × 조건 {0, 5} 재생                      → <root>/cmp_state/F#/{PF,MF,SU}{0,5}
#   3) eval    평가 시각 예측 보정 평가, N회 평균 ± 표준편차          → <root>/cmp_state/metrics_mean_std.csv
#   4) figs    논문 그림 4·5 형식                                       → <root>/figs/
#
# 사용:  paper_replicate.sh [단계...]     (기본: record replay eval figs 전부)
#   환경변수: ROOT(기본 ~/drone_project/flight_logs/indoor/paper_indoor), N(기본 5),
#             H(LiDAR 높이, 기본 1.2), MODEL(기본 x500_cf2x2 = Tello급)
#   QGroundControl이 켜져 있어야 한다 (record 단계).
#
# 조건은 논문 one_replay_state.sh 와 같다 — LiDAR2 외재 파라미터에만 오차를 준다.
#   0  오차 없음
#   5  위치 +0.10 / +0.10 / +0.05 m, roll·pitch +0.25°, yaw +0.20°
# 구조별 설정도 같다: PF cloud+process_on_arrival, MF position+process_on_arrival, SU sequential.
set -eo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
DPS=$HOME/drone_project/DronePositioningSystem
ROOT=${ROOT:-$HOME/drone_project/flight_logs/indoor/paper_indoor}
N=${N:-5}
H=${H:-1.2}
MODEL=${MODEL:-x500_cf2x2}
PARAMS=$DPS/ros2_ws/src/drone_localization/params/lidar_localization_indoor.yaml
STAGES=${*:-record replay eval figs}

record() {
    for i in $(seq "$N"); do
        if [ -d "$ROOT/flights/F$i/bag" ]; then echo "F$i 녹화 있음 — 건너뜀"; continue; fi
        RUN_NAME=F$i RECORD_BAG=1 MODEL=$MODEL HEADLESS=1 "$HERE/run_height_test.sh" "$H" "$ROOT/flights"
        sleep 3
    done
}

# 조건별 LiDAR -p 인자: 위치는 그 비행의 world.sdf, 조건 5는 LiDAR2에 오차를 더한다
lidar_args() {   # lidar_args <world.sdf> <조건>
    python3 - "$1" "$2" <<'EOF'
import sys, xml.etree.ElementTree as ET
sdf, cond = sys.argv[1], sys.argv[2]
p = {}
for m in ET.parse(sdf).getroot().iter('model'):
    n = m.get('name', '')
    if n.startswith('ground_lidar_'):
        i = n.rsplit('_', 1)[1]
        for k, v in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), map(float, m.find('pose').text.split())):
            p[f'lidar{i}_{k}'] = v
if cond == '5':
    for k, d in (('x', 0.10), ('y', 0.10), ('z', 0.05),
                 ('roll', 0.0043633), ('pitch', 0.0043633), ('yaw', 0.0034907)):
        p[f'lidar2_{k}'] += d
    p['warn_nominal_extrinsics'] = 'false'
print(' '.join(f'-p {k}:={v}' for k, v in sorted(p.items())))
EOF
}

replay() {
    source /opt/ros/humble/setup.bash
    source "$HOME/drone_project/ros2_ws/install/setup.bash"
    source "$DPS/ros2_ws/install/setup.bash"
    for F in "$ROOT"/flights/F*; do
        [ -d "$F/bag" ] || continue
        f=$(basename "$F")
        world=$(python3 -c "import xml.etree.ElementTree as E; print(E.parse('$F/world.sdf').getroot().find('world').get('name'))")
        for C in 0 5; do
            read -ra L <<<"$(lidar_args "$F/world.sdf" "$C")"
            for M in PF MF SU; do
                case $M in
                    PF) X=(-p fusion_mode:=cloud -p process_on_arrival:=true) ;;
                    MF) X=(-p fusion_mode:=position -p process_on_arrival:=true) ;;
                    SU) X=(-p fusion_mode:=sequential) ;;
                esac
                out=$ROOT/cmp_state/$f/$M$C
                if [ -f "$out/state_details.csv" ]; then continue; fi
                rm -rf "$out"; mkdir -p "$out"
                echo "재생 $f $M$C"
                python3 -m drone_localization.replay_bag --bag "$F/bag" --out "$out" --tracker cov \
                    --ground-truth-topic "/world/$world/dynamic_pose/info" --require-ground-truth --quiet \
                    --ros-args --params-file "$PARAMS" "${L[@]}" "${X[@]}" >"$out/replay.log" 2>&1
            done
        done
    done
}

for s in $STAGES; do
    case $s in
        record) record ;;
        replay) replay ;;
        eval)   python3 "$HERE/paper_eval.py" "$ROOT" ;;
        figs)   python3 "$HERE/paper_eval.py" "$ROOT" --figs ;;
        *) echo "알 수 없는 단계: $s (record|replay|eval|figs)" >&2; exit 1 ;;
    esac
done
