#!/usr/bin/env python3
"""
실내 케이지 경로 5종 비행 (실기 비교용).

    ros2 run drone_mission indoor_shapes_mission <helix|circle|figure8|square|backforth>

필드는 (0,0)~(3,2) m. 모든 비행은 원점에서 이륙해 시작점까지 0.4 m/s 로 가서 2.0초 멈춘 뒤
경로를 돌고, 1.0초 멈춘 다음 그 자리에 착륙한다.

위치만 지령하면 PX4 위치 제어기가 속도에 비례해 뒤처져 빠른 왕복에서 최고 속도가 나오지
않고, 속도만 더하면 제동이 늦어 양 끝에서 약 0.5 m 지나친다. 그래서 각 setpoint 에 속도와
가속도 피드포워드를 함께 보낸다.
"""

import math
import sys

import rclpy

from px4_msgs.msg import TrajectorySetpoint

from drone_mission.competition_mission import world_to_ned
from drone_mission.indoor_survey_mission import DT, IndoorSurveyMission, YAW

TRANSIT_SPEED = 0.4     # 이륙·시작점 이동 [m/s]
TAKEOFF_Z = 0.1
SHAPES = ('helix', 'circle', 'figure8', 'square', 'backforth')


def hold(path, seconds):
    path.extend([path[-1]] * int(round(seconds / DT)))


def line(path, end, speed, accel=None):
    """현재 점에서 end 까지 직선. accel 이 있으면 사다리꼴 속도, 없으면 등속."""
    start = path[-1]
    dist = math.dist(start, end)
    if dist < 1e-9:
        return
    if accel is None:
        n = max(1, int(round(dist / (speed * DT))))
        s_list = [dist * (i + 1) / n for i in range(n)]
    else:
        vmax = min(speed, math.sqrt(accel * dist))
        ta = vmax / accel
        total = dist / vmax + ta
        n = max(1, int(round(total / DT)))
        s_list = []
        for i in range(1, n + 1):
            t = total * i / n
            if t < ta:
                s = 0.5 * accel * t * t
            elif t < total - ta:
                s = 0.5 * accel * ta * ta + vmax * (t - ta)
            else:
                s = dist - 0.5 * accel * (total - t) ** 2
            s_list.append(s)
    for s in s_list:
        f = s / dist
        path.append(tuple(start[k] + (end[k] - start[k]) * f for k in range(3)))


def arc(path, cx, cy, r, a0, sweep, speed, z0=None, z1=None):
    """중심 (cx, cy), 시작각 a0, 회전각 sweep(부호 = 방향) 등속 원호. z 는 z0 → z1 직선."""
    n = max(1, int(round(abs(sweep) * r / (speed * DT))))
    z0 = path[-1][2] if z0 is None else z0
    z1 = z0 if z1 is None else z1
    for i in range(1, n + 1):
        f = i / n
        a = a0 + sweep * f
        path.append((cx + r * math.cos(a), cy + r * math.sin(a), z0 + (z1 - z0) * f))


def go_to_start(start):
    """원점 이륙 → 고도 상승 → 시작점 이동 → 2.0초 정지."""
    path = [(0.0, 0.0, TAKEOFF_Z)]
    line(path, (0.0, 0.0, start[2]), TRANSIT_SPEED)
    hold(path, 1.0)
    line(path, start, TRANSIT_SPEED)
    hold(path, 2.0)
    return path


def build_path(shape):
    """world 좌표 (x, y, z) setpoint 목록, 간격 DT."""
    if shape == 'helix':
        path = go_to_start((1.5, 0.2, 0.6))
        arc(path, 1.5, 1.0, 0.8, -math.pi / 2, 4 * math.pi, 0.4, 0.6, 1.4)
    elif shape == 'circle':
        path = go_to_start((1.5, 0.0, 1.0))
        arc(path, 1.5, 1.0, 1.0, -math.pi / 2, 4 * math.pi, 0.4)
    elif shape == 'figure8':
        path = go_to_start((1.5, 1.0, 1.0))
        for _ in range(2):      # 왼쪽 원 반시계, 오른쪽 원 시계 — (1.5, 1.0) 에서 접선이 이어진다
            arc(path, 0.8, 1.0, 0.7, 0.0, 2 * math.pi, 0.4)
            arc(path, 2.2, 1.0, 0.7, math.pi, -2 * math.pi, 0.4)
    elif shape == 'square':
        path = go_to_start((0.0, 0.0, 1.2))
        for _ in range(2):
            for corner in ((3.0, 0.0), (3.0, 2.0), (0.0, 2.0), (0.0, 0.0)):
                line(path, (*corner, 1.2), 0.7, accel=1.5)
                hold(path, 3.0)
    elif shape == 'backforth':
        path = go_to_start((0.0, 1.0, 1.2))
        for _ in range(3):
            for x in (3.0, 0.0):
                line(path, (x, 1.0, 1.2), 1.8, accel=1.5)
                hold(path, 1.0)
    else:
        raise ValueError(f'모르는 경로: {shape} (가능: {", ".join(SHAPES)})')
    hold(path, 1.0)
    return path


def velocities(path):
    """중앙 차분 world 속도. 이어지는 점의 차이로 구하므로 정지 구간은 0 이다."""
    out = []
    for i in range(len(path)):
        a, b = path[max(i - 1, 0)], path[min(i + 1, len(path) - 1)]
        dt = DT * (min(i + 1, len(path) - 1) - max(i - 1, 0))
        out.append(tuple((b[k] - a[k]) / dt for k in range(3)))
    return out


def accelerations(vel):
    """속도 목록의 중앙 차분 가속도."""
    return velocities(vel)


class IndoorShapesMission(IndoorSurveyMission):
    def __init__(self, shape):
        super().__init__()
        world = build_path(shape)
        self.setpoints = [world_to_ned(x, y, z, YAW) for x, y, z in world]
        # world ENU → PX4 NED 는 위치와 같은 축 교환이다 (x, y, z) → (y, x, -z)
        vel = velocities(world)
        ned = lambda v: (v[1], v[0], -v[2])
        self.ff_vel = [ned(v) for v in vel]
        self.ff_acc = [ned(a) for a in accelerations(vel)]
        self.get_logger().info(
            f'{shape}: setpoint {len(world)}개 ({len(world) * DT:.0f}초)')

    def publish_position_setpoint(self, x, y, z, yaw) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        if self.state == 'PLAYBACK':
            msg.velocity = [float(v) for v in self.ff_vel[self.playback_index]]
            msg.acceleration = [float(a) for a in self.ff_acc[self.playback_index]]
        else:
            msg.velocity = [math.nan, math.nan, math.nan]
            msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)


def main(args=None) -> None:
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    if len(argv) != 2 or argv[1] not in SHAPES:
        sys.exit(f'사용: indoor_shapes_mission <{"|".join(SHAPES)}>')
    rclpy.init(args=args)
    node = IndoorShapesMission(argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
