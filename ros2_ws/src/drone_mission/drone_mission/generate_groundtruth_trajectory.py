#!/usr/bin/env python3
"""
제24회 한국로봇항공기경연대회 중급부문 "실내 조난자 탐색 임무" ground-truth 궤적 생성기.

== 용도 ==
이 궤적은 실제 드론 비행 제어 명령이 아니다. 외부에 고정 설치된 LiDAR
(InnovizOne, Ouster OS0)-카메라 융합 측위 시스템이 추정한 드론 위치/자세가
얼마나 정확한지 비교 평가하기 위한 "정답(ground truth)" 시계열 데이터를
만드는 것이 유일한 목적이다. 측위 시스템의 추정 결과(estimated x,y,z,yaw)를
이 스크립트가 만든 CSV/Path와 나란히 두고 오차를 계산하는 데 사용한다.

== 좌표계 ==
competition_map.sdf의 world 좌표계를 그대로 사용한다 (원점(0,0)=home_pad,
단위 m, 평면 XY + 고도 Z, yaw는 world Z축 기준 라디안, X축 양의 방향이 0).

== 비행 시나리오 ==
1) 이륙: (0,0,0) -> (0,0,ALTITUDE) 수직 상승
2) 1차 탐색(라인트레이싱): HOME -> id1 -> id2 -> id3 -> id4, 두 지점 사이는
   반드시 격자선만 따라가는 L자(맨해튼) 경로로 분해. 각 마커에서 HOVER_DURATION초 호버링
3) 2차 구조 경로(직선): id4 -> id3 -> id2 -> id1 -> HOME, 지점간 직선 이동.
   각 마커에서 HOVER_DURATION초 호버링
4) 착륙: (0,0,ALTITUDE) -> (0,0,0) 수직 하강

모든 이동 구간은 같은 가속도 ACCEL로 가속·감속하는 사다리꼴 속도 프로파일을
따른다(구간이 짧아 최고속도에 못 닿으면 삼각형). 마커 배치는 LAYOUTS 중 하나를
고르며(--layout), 배치가 달라도 속도·가속 조건은 같다.

전체 궤적은 "전역 샘플링 클록"(0, DT, 2*DT, ...)으로 샘플링되므로, 이착륙/이동/호버링
구간 경계와 무관하게 타임스탬프 간격이 항상 정확히 DT로 균일하다.
"""

import argparse
import csv
import math
import os
from typing import Dict, List, NamedTuple, Optional, Tuple

# ===========================================================================
# 파라미터 (필요에 따라 조정)
# ===========================================================================
ALTITUDE = 2.0              # 비행 고도 (m) — 대회 규정 TBR 값
VERTICAL_SPEED = 0.5        # 이착륙 수직 속도 (m/s)
HORIZONTAL_SPEED = 1.5      # 수평 이동 속도 (m/s) — 규정집에 속도 제한 없음(확인함).
                             # 빠를수록 Line Tracing 채점(고도침하/구획선이탈, 70점)엔
                             # 불리해질 여지가 있으니 참고.
ACCEL = 1.0                 # 모든 이동 구간의 가속·감속 크기 (m/s²)
DT = 0.1                    # 샘플링 주기 (초). 측위 시스템 샘플링레이트에 맞춰 0.05~0.1 권장
HOVER_DURATION = 4.0        # 마커 도착 시 호버링 시간 (초) — LiDAR가 정지 인식하고
                             # 정밀 위치를 낼 시간을 벌기 위해 2초→4초로 늘림

FINAL_SETTLE_DURATION = 3.0  # 홈 복귀 후 착륙 전 제자리 호버(초). 이동 관성을 멈추고 yaw를 0으로 되돌린다

SMOOTH_YAW_TURNS = False    # True면 코너마다 짧은 제자리 회전 구간을 별도로 삽입해 yaw를 보간
YAW_TURN_DURATION = 0.5     # SMOOTH_YAW_TURNS=True일 때 회전에 걸리는 시간 (초)

EPS = 1e-9

# ===========================================================================
# 맵 좌표 (competition_map.sdf 와 동일)
# ===========================================================================
# 지침서 v03: 경기장 24x15, 격자 3m. 격자선이 전부 정수로 떨어진다
# (이전 판 30x20 / 7칸은 4.2857m라 소수였다).
GRID_X = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0]
GRID_Y = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0]
HOME = (0.0, 0.0)
# 마커 번호는 방문 순서 그대로 1->2->3->4가 되도록 부여 (예전엔 0-인덱스라
# 방문순서가 2,1,0,3으로 헷갈렸음). 좌표는 이전 판의 배치 비율을 그대로
# 옮겨 3m 격자 교점에 맞춘 것 — generate_competition_map.py와 동일해야 한다.
MARKERS = {
    1: (3.0, 3.0),
    2: (3.0, 12.0),
    3: (21.0, 3.0),
    4: (9.0, 9.0),
}

# 논문 Gazebo 비교용 비행 5회의 마커 배치. 1은 위 MARKERS(월드의 ArUco 위치)이고,
# 2–5는 대회 규칙처럼 내부 격자 교차점에서 무작위로 뽑되 마커 간 거리 6 m 이상,
# 필드 서쪽(x≤6)과 동쪽(x≥18)에 각각 하나 이상 있도록 고른 것이다. 2와 4는 마커
# 하나를 바깥선 교차점 (24, 9)/(0, 9)로 바꿨다 — 가까운 센서까지도 14.4 m로, 격자
# 교차점 중 가장 먼 곳이라 원거리 관측을 포함하기 위해서다. LiDAR 측위는 바닥
# 마커를 쓰지 않으므로 월드의 ArUco 위치는 배치 1로 둔다.
LAYOUTS: Dict[int, Dict[int, Tuple[float, float]]] = {
    1: MARKERS,
    2: {1: (9.0, 3.0), 2: (3.0, 9.0), 3: (24.0, 9.0), 4: (15.0, 6.0)},
    3: {1: (9.0, 9.0), 2: (6.0, 3.0), 3: (12.0, 3.0), 4: (18.0, 3.0)},
    4: {1: (6.0, 9.0), 2: (0.0, 9.0), 3: (15.0, 3.0), 4: (18.0, 12.0)},
    5: {1: (6.0, 6.0), 2: (3.0, 12.0), 3: (18.0, 6.0), 4: (12.0, 9.0)},
}

# 1차 탐색(격자) 구간에서 호버 없이 지나가는 경유점. {배치: {마커 번호: [경유점...]}}이면
# 그 마커로 가기 전에 경유점들을 격자선을 따라 차례로 지난다.
# 배치 2는 남쪽 센서 기준 가장 먼 바깥선 교차점 (0, 6)을 지나간다.
VIA_POINTS: Dict[int, Dict[int, List[Tuple[float, float]]]] = {
    2: {2: [(0.0, 6.0)]},
}

PHASE2_ORDER = [1, 2, 3, 4]   # 1차 탐색(라인트레이싱) 방문 순서
PHASE3_ORDER = [3, 2, 1]      # 2차 구조(직선) 방문 순서, id4는 이미 1차 탐색 끝에서 방문함


class Primitive(NamedTuple):
    kind: str                       # 'move' | 'hover' | 'turn'
    p0: Tuple[float, float, float]
    p1: Tuple[float, float, float]
    duration: float
    yaw0: float
    yaw1: float
    constrained: bool               # 시각화용: 격자선 추종 구간이면 True
    speed: float = 0.0              # 'move'의 최고 속도. 0이면 등속 보간


def manhattan_corner(p_start: Tuple[float, float], p_end: Tuple[float, float]) -> Tuple[float, float]:
    """X축 먼저 이동 후 Y축 이동 규칙으로 L자 코너 좌표를 계산한다."""
    return (p_end[0], p_start[1])


def heading(p0: Tuple[float, float], p1: Tuple[float, float]) -> Optional[float]:
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    if abs(dx) < EPS and abs(dy) < EPS:
        return None
    return math.atan2(dy, dx)


def _angle_diff(a0: float, a1: float) -> float:
    """a0 -> a1 로 가는 최단 회전각 (-pi, pi]."""
    return math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))


def _dist3(p0: Tuple[float, float, float], p1: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((b - a) ** 2 for a, b in zip(p0, p1)))


def snap_duration(duration: float, dt: float = DT) -> float:
    """전역 샘플링 클록(0, dt, 2dt, ...)과 프리미티브 경계가 항상 정확히 맞아떨어지도록
    구간 길이를 dt의 정수배로 스냅한다. 안 하면 호버링 시작 시각이 dt 격자에서
    살짝 어긋나 경계 샘플이 누락되며 호버 구간이 dt만큼 짧게 측정되는 문제가 생긴다."""
    steps = max(1, round(duration / dt))
    return steps * dt


def trapezoid_duration(length: float, v_max: float, accel: float = ACCEL) -> float:
    """가속 → 등속 → 감속에 걸리는 시간. 짧아서 v_max에 못 닿으면 삼각형 프로파일."""
    if length < EPS:
        return 0.0
    if length >= v_max * v_max / accel:
        return length / v_max + v_max / accel
    return 2.0 * math.sqrt(length / accel)


def trapezoid_fraction(tau: float, length: float, v_max: float, accel: float = ACCEL) -> float:
    """시각 tau(구간 시작 기준)까지 이동한 거리의 비율 [0, 1]."""
    if length < EPS:
        return 1.0
    total = trapezoid_duration(length, v_max, accel)
    t_acc = min(v_max / accel, total / 2.0)
    v_peak = accel * t_acc
    if tau <= t_acc:
        s = 0.5 * accel * tau * tau
    elif tau <= total - t_acc:
        s = 0.5 * accel * t_acc * t_acc + v_peak * (tau - t_acc)
    else:
        rest = max(total - tau, 0.0)
        s = length - 0.5 * accel * rest * rest
    return min(max(s / length, 0.0), 1.0)


def _append_move(prims: List[Primitive], yaw: float, pos: Tuple[float, float],
                  nxt: Tuple[float, float], z: float, speed: float, constrained: bool):
    """현재 yaw 상태를 받아 이동 프리미티브(필요시 회전 프리미티브 포함)를 추가하고 새 yaw/위치를 반환."""
    h = heading(pos, nxt)
    if h is None:
        return yaw, pos
    if SMOOTH_YAW_TURNS and abs(_angle_diff(yaw, h)) > EPS:
        prims.append(Primitive('turn', (*pos, z), (*pos, z), snap_duration(YAW_TURN_DURATION), yaw, h, False))
        yaw = h
    p0, p1 = (*pos, z), (*nxt, z)
    duration = snap_duration(trapezoid_duration(_dist3(p0, p1), speed))
    prims.append(Primitive('move', p0, p1, duration, h, h, constrained, speed))
    return h, nxt


def build_primitives(markers: Optional[Dict[int, Tuple[float, float]]] = None,
                     via: Optional[Dict[int, List[Tuple[float, float]]]] = None) -> List[Primitive]:
    markers = MARKERS if markers is None else markers
    via = {} if via is None else via
    prims: List[Primitive] = []
    yaw = 0.0  # 이륙 직후 첫 라인트레이싱 구간이 +X 방향이라 0으로 시작
    climb = snap_duration(trapezoid_duration(ALTITUDE, VERTICAL_SPEED))

    # 단계 1: 자동 이륙 (z만 0 -> ALTITUDE)
    prims.append(Primitive('move', (0.0, 0.0, 0.0), (0.0, 0.0, ALTITUDE),
                            climb, yaw, yaw, False, VERTICAL_SPEED))

    # 단계 2: 1차 탐색 (라인트레이싱, 격자선만 사용)
    pos = HOME
    for marker_id in PHASE2_ORDER:
        dest = markers[marker_id]
        for target in [*via.get(marker_id, []), dest]:
            corner = manhattan_corner(pos, target)
            for nxt in (corner, target):
                yaw, pos = _append_move(prims, yaw, pos, nxt, ALTITUDE, HORIZONTAL_SPEED, True)
        prims.append(Primitive('hover', (*pos, ALTITUDE), (*pos, ALTITUDE),
                                snap_duration(HOVER_DURATION), yaw, yaw, None))

    # 단계 3: 2차 구조 경로 (직선, id4 -> id3 -> id2 -> id1 -> HOME)
    for marker_id in PHASE3_ORDER:
        dest = markers[marker_id]
        yaw, pos = _append_move(prims, yaw, pos, dest, ALTITUDE, HORIZONTAL_SPEED, False)
        prims.append(Primitive('hover', (*pos, ALTITUDE), (*pos, ALTITUDE),
                                snap_duration(HOVER_DURATION), yaw, yaw, None))
    yaw, pos = _append_move(prims, yaw, pos, HOME, ALTITUDE, HORIZONTAL_SPEED, False)
    # 도착 직후 바로 착륙하면 추종 지연으로 남은 속도 때문에 패드에서 밀려 내려앉고,
    # 마지막 구간 방향(대각선)으로 돌아간 채 착지해 다음 비행의 출발 조건이 달라진다.
    prims.append(Primitive('hover', (*HOME, ALTITUDE), (*HOME, ALTITUDE),
                            snap_duration(FINAL_SETTLE_DURATION), yaw, 0.0, None))
    yaw = 0.0

    # 단계 4: 자동 착륙 (z만 ALTITUDE -> 0)
    prims.append(Primitive('move', (0.0, 0.0, ALTITUDE), (0.0, 0.0, 0.0),
                            climb, yaw, yaw, False, VERTICAL_SPEED))

    return prims


class Sample(NamedTuple):
    t: float
    x: float
    y: float
    z: float
    yaw: float
    constrained: Optional[bool]   # None=호버/이착륙, True=격자추종, False=직선이동


def sample_trajectory(prims: List[Primitive], dt: float = DT) -> List[Sample]:
    starts = []
    t_cursor = 0.0
    for p in prims:
        starts.append(t_cursor)
        t_cursor += p.duration
    total_duration = t_cursor

    n_total = round(total_duration / dt)
    samples: List[Sample] = []
    pi = 0
    for i in range(n_total + 1):
        t = min(i * dt, total_duration)
        while pi < len(prims) - 1 and t > starts[pi] + prims[pi].duration + EPS:
            pi += 1
        prim = prims[pi]
        local_dur = prim.duration
        frac = 0.0 if local_dur < EPS else (t - starts[pi]) / local_dur
        frac = min(max(frac, 0.0), 1.0)
        pos_frac = frac
        if prim.kind == 'move' and prim.speed > 0.0:
            # snap_duration으로 dt 배수가 된 구간 길이에 맞춰 프로파일 시간을 늘이거나 줄인다.
            length = _dist3(prim.p0, prim.p1)
            raw = trapezoid_duration(length, prim.speed)
            pos_frac = trapezoid_fraction(frac * raw, length, prim.speed)

        x = prim.p0[0] + (prim.p1[0] - prim.p0[0]) * pos_frac
        y = prim.p0[1] + (prim.p1[1] - prim.p0[1]) * pos_frac
        z = prim.p0[2] + (prim.p1[2] - prim.p0[2]) * pos_frac
        yaw = prim.yaw0 + _angle_diff(prim.yaw0, prim.yaw1) * frac

        constrained = prim.constrained if prim.kind == 'move' else None
        samples.append(Sample(t, x, y, z, yaw, constrained))

    return samples


def write_csv(samples: List[Sample], path: str) -> None:
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "x", "y", "z", "yaw"])
        for s in samples:
            writer.writerow([f"{s.t:.4f}", f"{s.x:.6f}", f"{s.y:.6f}", f"{s.z:.6f}", f"{s.yaw:.6f}"])


def plot_preview(samples: List[Sample], png_path: str,
                 markers: Optional[Dict[int, Tuple[float, float]]] = None) -> None:
    markers = MARKERS if markers is None else markers
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 7))

    for x in GRID_X:
        ax.plot([x, x], [GRID_Y[0], GRID_Y[-1]], color="gray", linewidth=1, zorder=1)
    for y in GRID_Y:
        ax.plot([GRID_X[0], GRID_X[-1]], [y, y], color="gray", linewidth=1, zorder=1)

    phase2_xy = [(s.x, s.y) for s in samples if s.constrained is True]
    phase3_xy = [(s.x, s.y) for s in samples if s.constrained is False
                 and not (abs(s.x) < EPS and abs(s.y) < EPS)]

    if phase2_xy:
        xs, ys = zip(*phase2_xy)
        ax.plot(xs, ys, color="blue", linewidth=2, label="Phase 2: line-tracing search", zorder=3)
    if phase3_xy:
        xs, ys = zip(*phase3_xy)
        ax.plot(xs, ys, color="red", linewidth=2, linestyle="--", label="Phase 3: straight-line rescue path", zorder=2)

    for marker_id, (mx, my) in markers.items():
        ax.scatter([mx], [my], marker="s", s=140, color="black", zorder=5)
        ax.annotate(f"id{marker_id}", (mx, my), textcoords="offset points",
                    xytext=(8, 8), fontsize=9, zorder=6)
        ax.scatter([mx], [my], marker="o", s=300, facecolors="none",
                   edgecolors="orange", linewidths=2, zorder=4)

    ax.scatter([HOME[0]], [HOME[1]], marker="^", s=150, color="green", zorder=5)
    ax.annotate("HOME", HOME, textcoords="offset points", xytext=(8, -14), fontsize=9)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Ground-truth trajectory preview (competition_map)")
    ax.set_xlim(-2, 32)
    ax.set_ylim(-2, 22)
    ax.set_aspect("equal")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    print(f"미리보기 plot 저장: {png_path}")


def generate_trajectory(dt: float = DT, layout: int = 1) -> List[Sample]:
    """다른 모듈(ROS2 노드 등)에서 import해서 쓰는 진입점."""
    return sample_trajectory(build_layout_primitives(layout), dt)


def build_layout_primitives(layout: int) -> List[Primitive]:
    return build_primitives(LAYOUTS[layout], VIA_POINTS.get(layout))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--layout', type=int, choices=sorted(LAYOUTS), default=None,
                        help='마커 배치 번호. 생략하면 모든 배치를 만든다.')
    args = parser.parse_args()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    for layout in ([args.layout] if args.layout else sorted(LAYOUTS)):
        samples = generate_trajectory(layout=layout)
        stem = os.path.join(out_dir, f"groundtruth_trajectory_L{layout}")
        write_csv(samples, stem + ".csv")
        plot_preview(samples, stem + "_preview.png", LAYOUTS[layout])
        print(f"배치 {layout}: 총 비행시간 {samples[-1].t:.1f}초, 샘플 {len(samples)}개, dt={DT}s")


if __name__ == "__main__":
    main()
