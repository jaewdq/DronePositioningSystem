#!/usr/bin/env python3
"""
대회 임무 비행 제어 노드.

generate_groundtruth_trajectory.py가 만드는 것과 완전히 동일한 좌표 시퀀스
(HOME 이륙 -> 격자선 L자 경로로 id2->id1->id0->id3 순회(각 2초 호버) ->
id3->id0->id1->id2->HOME 직선 복귀(각 2초 호버))를 PX4 Offboard 모드로
실제로 비행시킨다.

주의: 카메라로 바닥 라인을 보고 따라가거나 ArUco 마커를 인식해서 위치를
찾는 비전 인식은 하지 않는다. 마커 좌표를 이미 알고 있다는 가정 하에
offboard 위치 제어로 좌표를 그대로 통과시키기만 한다.

좌표계 변환: competition_map.sdf의 world 좌표(ENU, Z-up, 즉 X/Y 수평,
Z=양수가 위)를 PX4 local NED(X=North, Y=East, Z=Down)로 변환해서 보낸다.
변환식은 PX4-Autopilot의 GZBridge.cpp에서 실제로 쓰는 것과 동일하다:
    NED_x = world_y, NED_y = world_x, NED_z = -world_z
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
    VehicleLocalPosition,
    VehicleStatus,
)

from drone_mission.generate_groundtruth_trajectory import build_primitives, sample_trajectory, DT


def world_to_ned(x: float, y: float, z: float, yaw: float):
    """Gazebo world(ENU, Z-up) -> PX4 local NED. GZBridge.cpp의 변환식과 동일."""
    ned_x = y
    ned_y = x
    ned_z = -z
    # heading 벡터도 같은 축 치환을 받으므로: ned_yaw = atan2(world_dx, world_dy) = pi/2 - world_yaw
    ned_yaw = math.atan2(math.sin(math.pi / 2 - yaw), math.cos(math.pi / 2 - yaw))
    return ned_x, ned_y, ned_z, ned_yaw


class CompetitionMission(Node):
    def __init__(self) -> None:
        super().__init__('competition_mission')

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
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback, qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback, qos_profile)

        # ground-truth와 동일한 경로 생성. 마지막 착륙 프리미티브는 빼고
        # PX4 자체 NAV_LAND 명령으로 착륙시킨다 (offboard로 직접 z=0까지
        # 밀어붙이는 것보다 지면 접촉/disarm 처리가 안전함).
        prims = build_primitives()[:-1]
        world_samples = sample_trajectory(prims, DT)
        self.setpoints = [world_to_ned(s.x, s.y, s.z, s.yaw) for s in world_samples]
        self.get_logger().info(f'재생할 setpoint {len(self.setpoints)}개 생성 (NED 변환 완료)')

        self.tick = 0
        self.last_arm_attempt_tick = None
        self.playback_index = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # ARMING -> PLAYBACK -> LAND -> DONE
        self.state = 'ARMING'

        self.timer = self.create_timer(DT, self.timer_callback)

    def local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 명령 전송')

    def engage_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 모드 전환')

    def land(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('착륙 명령 전송')

    def publish_offboard_control_heartbeat_signal(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()
        self.tick += 1

        is_offboard = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        is_armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

        # armed+offboard가 확인될 때까지 1초 간격으로 계속 재시도 (첫 시도가
        # GCS연결/EKF준비 타이밍에 거부되면 한 번만 쏘고 끝나는 구조는 영영 막히기 때문)
        if self.state == 'ARMING' and self.tick >= 10 and not (is_offboard and is_armed):
            if self.last_arm_attempt_tick is None or self.tick - self.last_arm_attempt_tick >= 10:
                self.engage_offboard_mode()
                self.arm()
                self.last_arm_attempt_tick = self.tick

        if self.state == 'ARMING':
            x, y, z, yaw = self.setpoints[0]
            self.publish_position_setpoint(x, y, z, yaw)
            if is_offboard and is_armed:
                self.get_logger().info('Armed + Offboard 진입 완료 -> 궤적 재생 시작')
                self.state = 'PLAYBACK'
                self.playback_index = 0

        elif self.state == 'PLAYBACK':
            x, y, z, yaw = self.setpoints[self.playback_index]
            self.publish_position_setpoint(x, y, z, yaw)
            if self.playback_index % 100 == 0:
                self.get_logger().info(
                    f'재생 {self.playback_index}/{len(self.setpoints)} '
                    f'(NED x={x:.2f} y={y:.2f} z={z:.2f})')
            self.playback_index += 1
            if self.playback_index >= len(self.setpoints):
                self.get_logger().info('궤적 재생 완료 -> 착륙 시작')
                self.state = 'LAND'

        elif self.state == 'LAND':
            self.land()
            self.state = 'DONE'

        elif self.state == 'DONE':
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('착륙 및 disarm 완료. 노드를 종료합니다.')
                sys.exit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompetitionMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
