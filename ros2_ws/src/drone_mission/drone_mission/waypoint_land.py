#!/usr/bin/env python3
"""PX4 오프보드 미션: (0,0,3)로 이륙 -> 2초 호버링 -> 전방 5m 이동 -> 2초 호버링 -> 착륙."""

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


class WaypointLand(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_land')

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

        # 이 PX4 빌드는 메시지 버전 접미사가 붙은 토픽 이름을 사용함
        # (ros2 topic list 로 직접 확인한 실제 이름: vehicle_local_position_v1, vehicle_status_v4)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback, qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback, qos_profile)

        # NED 프레임: z는 아래쪽이 양수라서 고도 3m -> z=-3
        self.takeoff_z = -3.0
        self.forward_x = 5.0
        self.position_tolerance = 0.3
        self.hover_duration = 2.0  # 초

        self.tick = 0
        self.last_arm_attempt_tick = None
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # TAKEOFF -> HOVER1 -> FORWARD -> HOVER2 -> LAND -> DONE
        self.state = 'TAKEOFF'
        self.hover_start_time = None

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

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

    def publish_position_setpoint(self, x: float, y: float, z: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
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

    def reached(self, x: float, y: float, z: float) -> bool:
        p = self.vehicle_local_position
        return (
            abs(p.x - x) < self.position_tolerance
            and abs(p.y - y) < self.position_tolerance
            and abs(p.z - z) < self.position_tolerance
        )

    def hover_elapsed(self) -> float:
        return (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()
        self.tick += 1

        is_offboard = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        is_armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

        # 오프보드 setpoint 스트림이 1초(워밍업) 돈 뒤부터, armed+offboard가
        # 확인될 때까지 1초 간격으로 계속 재시도한다. (GCS 연결/EKF 준비가
        # 늦어서 첫 시도가 거부되면 한 번만 쏘고 끝나는 구조는 영영 막히기 때문)
        # TAKEOFF 단계에서 한 번 성공한 뒤로는 다시 시도하지 않는다 — 안 그러면
        # 착륙 중 nav_state가 AUTO_LAND로 바뀌는 걸 "offboard 아님"으로 오인해서
        # 착륙 도중에 또 arm/offboard를 재전송해버린다.
        if self.state == 'TAKEOFF' and self.tick >= 10 and not (is_offboard and is_armed):
            if self.last_arm_attempt_tick is None or self.tick - self.last_arm_attempt_tick >= 10:
                self.engage_offboard_mode()
                self.arm()
                self.last_arm_attempt_tick = self.tick

        if self.state == 'TAKEOFF':
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_z)
            if is_offboard and self.reached(0.0, 0.0, self.takeoff_z):
                self.get_logger().info('이륙 완료 (0,0,3) -> 2초 호버링')
                self.state = 'HOVER1'
                self.hover_start_time = self.get_clock().now()

        elif self.state == 'HOVER1':
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_z)
            if self.hover_elapsed() >= self.hover_duration:
                self.get_logger().info('호버링 종료 -> 전방 5m 이동 시작')
                self.state = 'FORWARD'

        elif self.state == 'FORWARD':
            self.publish_position_setpoint(self.forward_x, 0.0, self.takeoff_z)
            if is_offboard and self.reached(self.forward_x, 0.0, self.takeoff_z):
                self.get_logger().info('목표 지점 도착 -> 2초 호버링')
                self.state = 'HOVER2'
                self.hover_start_time = self.get_clock().now()

        elif self.state == 'HOVER2':
            self.publish_position_setpoint(self.forward_x, 0.0, self.takeoff_z)
            if self.hover_elapsed() >= self.hover_duration:
                self.get_logger().info('호버링 종료 -> 착륙 시작')
                self.state = 'LAND'

        elif self.state == 'LAND':
            self.land()
            self.state = 'DONE'

        elif self.state == 'DONE':
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('착륙 및 disarm 완료. 노드를 종료합니다.')
                # rclpy.shutdown()을 콜백 안에서 호출하면 spin 루프가 깔끔히
                # 끝나지 않고 좀비로 남는 경우가 있어, 프로세스를 직접 종료한다.
                sys.exit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointLand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
