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

전체 궤적은 "전역 샘플링 클록"(0, DT, 2*DT, ...)으로 샘플링되므로, 이착륙/이동/호버링
구간 경계와 무관하게 타임스탬프 간격이 항상 정확히 DT로 균일하다.
"""

import csv
import math
import os
from typing import List, NamedTuple, Optional, Tuple

# ===========================================================================
# 파라미터 (필요에 따라 조정)
# ===========================================================================
ALTITUDE = 2.0              # 비행 고도 (m) — 대회 규정 TBR 값
VERTICAL_SPEED = 0.5        # 이착륙 수직 속도 (m/s)
HORIZONTAL_SPEED = 0.75     # 수평 이동 속도 (m/s), 규정 권장 범위 0.5~1.0 m/s
DT = 0.1                    # 샘플링 주기 (초). 측위 시스템 샘플링레이트에 맞춰 0.05~0.1 권장
HOVER_DURATION = 2.0        # 마커 도착 시 호버링 시간 (초)

SMOOTH_YAW_TURNS = False    # True면 코너마다 짧은 제자리 회전 구간을 별도로 삽입해 yaw를 보간
YAW_TURN_DURATION = 0.5     # SMOOTH_YAW_TURNS=True일 때 회전에 걸리는 시간 (초)

EPS = 1e-9

# ===========================================================================
# 맵 좌표 (competition_map.sdf 와 동일)
# ===========================================================================
GRID_X = [0.0, 4.2857, 8.5714, 12.8571, 17.1429, 21.4286, 25.7143, 30.0]
GRID_Y = [0.0, 4.0, 8.0, 12.0, 16.0, 20.0]
HOME = (0.0, 0.0)
# 마커 번호는 방문 순서 그대로 1->2->3->4가 되도록 부여 (예전엔 0-인덱스라
# 방문순서가 2,1,0,3으로 헷갈렸음). id4는 원래 (25.7143,16.0)였으나
# id1->id0(현재 id2->id3) 구간 코너와 좌표가 겹쳐서 (12.8571,12.0)로 재배치.
MARKERS = {
    1: (4.2857, 4.0),
    2: (4.2857, 16.0),
    3: (25.7143, 4.0),
    4: (12.8571, 12.0),
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
    duration = snap_duration(_dist3(p0, p1) / speed)
    prims.append(Primitive('move', p0, p1, duration, h, h, constrained))
    return h, nxt


def build_primitives() -> List[Primitive]:
    prims: List[Primitive] = []
    yaw = 0.0  # 이륙 직후 첫 라인트레이싱 구간이 +X 방향이라 0으로 시작

    # 단계 1: 자동 이륙 (z만 0 -> ALTITUDE)
    prims.append(Primitive('move', (0.0, 0.0, 0.0), (0.0, 0.0, ALTITUDE),
                            snap_duration(ALTITUDE / VERTICAL_SPEED), yaw, yaw, False))

    # 단계 2: 1차 탐색 (라인트레이싱, 격자선만 사용)
    pos = HOME
    for marker_id in PHASE2_ORDER:
        dest = MARKERS[marker_id]
        corner = manhattan_corner(pos, dest)
        for nxt in (corner, dest):
            yaw, pos = _append_move(prims, yaw, pos, nxt, ALTITUDE, HORIZONTAL_SPEED, True)
        prims.append(Primitive('hover', (*pos, ALTITUDE), (*pos, ALTITUDE),
                                snap_duration(HOVER_DURATION), yaw, yaw, None))

    # 단계 3: 2차 구조 경로 (직선, id4 -> id3 -> id2 -> id1 -> HOME)
    for marker_id in PHASE3_ORDER:
        dest = MARKERS[marker_id]
        yaw, pos = _append_move(prims, yaw, pos, dest, ALTITUDE, HORIZONTAL_SPEED, False)
        prims.append(Primitive('hover', (*pos, ALTITUDE), (*pos, ALTITUDE),
                                snap_duration(HOVER_DURATION), yaw, yaw, None))
    yaw, pos = _append_move(prims, yaw, pos, HOME, ALTITUDE, HORIZONTAL_SPEED, False)

    # 단계 4: 자동 착륙 (z만 ALTITUDE -> 0)
    prims.append(Primitive('move', (0.0, 0.0, ALTITUDE), (0.0, 0.0, 0.0),
                            snap_duration(ALTITUDE / VERTICAL_SPEED), yaw, yaw, False))

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

        x = prim.p0[0] + (prim.p1[0] - prim.p0[0]) * frac
        y = prim.p0[1] + (prim.p1[1] - prim.p0[1]) * frac
        z = prim.p0[2] + (prim.p1[2] - prim.p0[2]) * frac
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


def plot_preview(samples: List[Sample], png_path: str) -> None:
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

    for marker_id, (mx, my) in MARKERS.items():
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


def generate_trajectory(dt: float = DT) -> List[Sample]:
    """다른 모듈(ROS2 노드 등)에서 import해서 쓰는 진입점."""
    prims = build_primitives()
    return sample_trajectory(prims, dt)


def main() -> None:
    out_dir = os.path.dirname(os.path.abspath(__file__))
    samples = generate_trajectory()

    csv_path = os.path.join(out_dir, "groundtruth_trajectory.csv")
    write_csv(samples, csv_path)
    print(f"CSV 저장: {csv_path} ({len(samples)}개 샘플)")

    png_path = os.path.join(out_dir, "groundtruth_trajectory_preview.png")
    plot_preview(samples, png_path)

    total_t = samples[-1].t
    print(f"총 비행시간: {total_t:.2f}초, 샘플 수: {len(samples)}, dt={DT}s")


if __name__ == "__main__":
    main()
