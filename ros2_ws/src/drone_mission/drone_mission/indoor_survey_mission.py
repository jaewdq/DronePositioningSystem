#!/usr/bin/env python3
"""
실내 케이지(indoor_cage) 커버리지 시험 비행.

필드(x 0~3, y 0~2 m, 원점 = 필드 남서 모서리 = 이륙 지점)를 지그재그로 훑어서
지상 LiDAR가 위치마다 기체를 얼마나 보는지 재기 위한 표적 비행이다.
  1) 원점에서 비행 고도까지 천천히 상승 (이륙 구간도 측정 대상)
  2) x = 0, 0.75, 1.5, 2.25, 3.0 세로줄을 y 0 → 2 → 0 … 으로 번갈아 왕복.
     각 줄 끝에서 TURN_HOLD_S 동안 호버한 뒤, 반원(반지름 = 줄 간격/2)으로 다음 줄로 돈다.
     반원 꼭대기가 y = 0 / 2 에 닿도록 직선 구간을 그만큼 줄인다 — 필드를 벗어나지 않는다.
  3) 마지막 줄 끝 (3, 2)에서 PX4 NAV_LAND로 착륙

비행 제어 구조는 competition_mission.py와 같다 (world ENU → PX4 NED 변환 포함).
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

from drone_mission.competition_mission import world_to_ned

DT = 0.1
SPEED = 0.4                       # 수평 이동 속도 [m/s]
CLIMB_SPEED = 0.25                # 수직 이동 속도 [m/s]
ALTITUDE = 1.0                    # 비행 고도 [m]
TAKEOFF_Z = 0.1                   # 첫 setpoint — 여기서부터 CLIMB_SPEED로 상승
SWEEP_XS = [0.0, 0.75, 1.5, 2.25, 3.0]   # 세로줄 x 위치
SWEEP_YS = (0.0, 2.0)             # 세로줄 y 범위
HOLD_S = 1.0                      # 이륙 상승 끝·도착점에서 멈추는 시간
TURN_HOLD_S = 3.0                 # 각 U턴 직전 호버 시간 (평가의 호버 구간)
ARC_STEPS = 12                    # 반원 하나를 나누는 직선 조각 수
YAW = 0.0                         # world 기준 기수 방향 (+x)


def build_waypoints():
    """(x, y, z, 이동속도, 정지 시간[s]) 목록. 원점 상승 → 세로줄 지그재그(U턴 전 호버,
    반원 회전) → (3, 2)."""
    r = (SWEEP_XS[1] - SWEEP_XS[0]) / 2.0
    y_lo, y_hi = SWEEP_YS
    wps = [(0.0, 0.0, TAKEOFF_Z, CLIMB_SPEED, 0.0),
           (0.0, 0.0, ALTITUDE, CLIMB_SPEED, HOLD_S)]
    up = True
    for k, x in enumerate(SWEEP_XS):
        last = k == len(SWEEP_XS) - 1
        # 첫 줄은 원점에서, 마지막 줄은 도착점까지 — 나머지는 반원과 이어지도록 r만큼 짧게
        end_y = (y_hi if up else y_lo) if last else ((y_hi - r) if up else (y_lo + r))
        wps.append((x, end_y, ALTITUDE, SPEED, HOLD_S if last else TURN_HOLD_S))
        if last:
            break
        # 다음 줄로 넘어가는 반원: 중심 (x + r, end_y), 왼쪽(180°)에서 오른쪽(0°)으로
        cx = x + r
        for i in range(1, ARC_STEPS + 1):
            a = math.pi * (1 - i / ARC_STEPS)
            wps.append((cx + r * math.cos(a), end_y + (r if up else -r) * math.sin(a),
                        ALTITUDE, SPEED, 0.0))
        up = not up
    return wps


def sample_waypoints(wps):
    """꼭짓점 사이를 등속 직선으로 DT 간격 샘플링. 정지 시간이 있는 꼭짓점에서만 그만큼 정지.
    등속을 유지하려고 이동 거리를 다음 조각으로 넘겨 가며 샘플을 찍는다."""
    out = [wps[0][:3]]
    carry = 0.0                                   # 이전 조각에서 남은 이동 거리
    for a, b in zip(wps, wps[1:]):
        pa, pb, v, hold = a[:3], b[:3], b[3], b[4]
        dist = math.dist(pa, pb)
        step = v * DT
        s = step - carry
        while s <= dist + 1e-9:
            f = s / dist if dist > 0 else 1.0
            out.append(tuple(pa[j] + (pb[j] - pa[j]) * f for j in range(3)))
            s += step
        carry = dist - (s - step)
        if hold > 0:
            if out[-1] != pb:
                out.append(pb)
            out.extend([pb] * int(round(hold / DT)))
            carry = 0.0
    return out


class IndoorSurveyMission(Node):
    def __init__(self) -> None:
        super().__init__('indoor_survey_mission')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback, qos_profile)

        world = sample_waypoints(build_waypoints())
        self.setpoints = [world_to_ned(x, y, z, YAW) for x, y, z in world]
        self.get_logger().info(
            f'setpoint {len(self.setpoints)}개 ({len(self.setpoints) * DT:.0f}초), '
            f'고도 {ALTITUDE} m')

        self.tick = 0
        self.last_arm_attempt_tick = None
        self.playback_index = 0
        self.vehicle_status = VehicleStatus()
        self.state = 'ARMING'           # ARMING -> PLAYBACK -> LAND -> DONE
        self.timer = self.create_timer(DT, self.timer_callback)

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def timer_callback(self) -> None:
        hb = OffboardControlMode()
        hb.position = True
        hb.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(hb)
        self.tick += 1

        is_offboard = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        is_armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

        if self.state == 'ARMING':
            self.publish_position_setpoint(*self.setpoints[0])
            if self.tick >= 10 and not (is_offboard and is_armed):
                if self.last_arm_attempt_tick is None or self.tick - self.last_arm_attempt_tick >= 10:
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                    self.last_arm_attempt_tick = self.tick
            if is_offboard and is_armed:
                self.get_logger().info('Armed + Offboard -> 비행 시작')
                self.state = 'PLAYBACK'

        elif self.state == 'PLAYBACK':
            self.publish_position_setpoint(*self.setpoints[self.playback_index])
            if self.playback_index % 100 == 0:
                self.get_logger().info(f'진행 {self.playback_index}/{len(self.setpoints)}')
            self.playback_index += 1
            if self.playback_index >= len(self.setpoints):
                self.state = 'LAND'

        elif self.state == 'LAND':
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info('착륙 명령 전송')
            self.state = 'DONE'

        elif self.state == 'DONE':
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('착륙 및 disarm 완료. 노드를 종료합니다.')
                sys.exit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IndoorSurveyMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
