#!/usr/bin/env bash
# 녹화한 실내 비행 bag 하나를 PF / MF / SU 세 융합 구조에 똑같이 재생한다.
# 입력이 완전히 같으므로 결과 차이는 전부 융합 구조 탓이다 (논문 cmp_state와 같은 방식).
#
# 사용:  compare_fusion.sh <run_height_test.sh 결과 폴더>
#   폴더 안에 bag/ (RECORD_BAG=1 로 녹화)와 world.sdf 가 있어야 한다.
#   결과: <폴더>/fusion/{PF,MF,SU}/  → plot_fusion.py <폴더>/fusion 으로 그림
#
# 구조별 설정은 논문 one_replay_state.sh 와 같다.
#   PF (점군 융합)  fusion_mode=cloud,      process_on_arrival=true
#   MF (관측 융합)  fusion_mode=position,   process_on_arrival=true
#   SU (순차 갱신)  fusion_mode=sequential
set -eo pipefail

RUN=${1:?사용: compare_fusion.sh <결과 폴더>}
RUN=$(cd "$RUN" && pwd)
DPS=$HOME/drone_project/DronePositioningSystem
PARAMS=$DPS/ros2_ws/src/drone_localization/params/lidar_localization_indoor.yaml
[ -d "$RUN/bag" ] && [ -f "$RUN/world.sdf" ] || { echo "$RUN 에 bag/ 와 world.sdf 가 없습니다 (RECORD_BAG=1 로 녹화)" >&2; exit 1; }

source /opt/ros/humble/setup.bash
source "$HOME/drone_project/ros2_ws/install/setup.bash"
source "$DPS/ros2_ws/install/setup.bash"

WORLD=$(python3 -c "import xml.etree.ElementTree as E; print(E.parse('$RUN/world.sdf').getroot().find('world').get('name'))")
read -ra LIDAR_ARGS <<<"$(python3 "$DPS/experiments/indoor/count_target_points.py" \
    --world-sdf "$RUN/world.sdf" --params "$PARAMS" --out /dev/null --print-tracker-args)"

for M in PF MF SU; do
    case $M in
        PF) F=(-p fusion_mode:=cloud -p process_on_arrival:=true) ;;
        MF) F=(-p fusion_mode:=position -p process_on_arrival:=true) ;;
        SU) F=(-p fusion_mode:=sequential) ;;
    esac
    OUT=$RUN/fusion/$M
    rm -rf "$OUT"; mkdir -p "$OUT"
    echo "재생 $M ..."
    python3 -m drone_localization.replay_bag --bag "$RUN/bag" --out "$OUT" --tracker cov \
        --ground-truth-topic "/world/$WORLD/dynamic_pose/info" --require-ground-truth --quiet \
        --ros-args --params-file "$PARAMS" "${LIDAR_ARGS[@]}" "${F[@]}" >"$OUT/replay.log" 2>&1
done
echo "완료 → $RUN/fusion  (그림: python3 $DPS/experiments/indoor/plot_fusion.py $RUN/fusion)"
