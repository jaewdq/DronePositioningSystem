#!/usr/bin/env python3
"""
generate_groundtruth_trajectory.py 가 만든 ground-truth 궤적을
RViz2에서 바로 볼 수 있도록 ROS2 토픽으로 발행하는 노드.

== 용도 ==
여기서 발행하는 경로는 실제 비행 제어 명령이 아니다. 외부 고정
LiDAR(InnovizOne, Ouster OS0)-카메라 융합 측위 시스템이 추정한 드론
위치/자세의 정확도를, 이 "정답(ground truth)" 경로와 비교 평가하기 위한
참고용 시각화 데이터다.

발행 토픽:
  /ground_truth_path     (nav_msgs/Path)              — 전체 궤적
  /ground_truth_markers  (visualization_msgs/MarkerArray) — ArUco 마커 4개 +
                                                             호버링 지점 표시

CSV(groundtruth_trajectory.csv)와 동일한 generate_trajectory() 결과를
그대로 사용하므로 두 출력은 항상 일치한다.

실행:
  ros2 run drone_mission ground_truth_path_publisher
  rviz2 에서 Fixed Frame을 'map'으로 두고 Path/MarkerArray 디스플레이 추가
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from drone_mission.generate_groundtruth_trajectory import (
    generate_trajectory, MARKERS, HOME, ALTITUDE,
)

FRAME_ID = "map"


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GroundTruthPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth_path_publisher')

        # RViz처럼 늦게 붙는 구독자도 마지막 메시지를 받을 수 있도록 TRANSIENT_LOCAL(래치) 사용
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.path_pub = self.create_publisher(Path, '/ground_truth_path', latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/ground_truth_markers', latched_qos)

        self.get_logger().info('ground-truth 궤적 생성 중...')
        self.samples = generate_trajectory()
        self.get_logger().info(f'{len(self.samples)}개 샘플 생성 완료, 발행 시작')

        self.publish_path()
        self.publish_markers()

        # 늦게 뜨는 RViz도 확실히 받도록 5초마다 재발행 (래치 QoS라 사실 1회로도 충분하지만 안전망)
        self.timer = self.create_timer(5.0, self.republish)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def publish_path(self) -> None:
        path = Path()
        path.header.frame_id = FRAME_ID
        path.header.stamp = self._stamp()

        for s in self.samples:
            pose = PoseStamped()
            pose.header.frame_id = FRAME_ID
            pose.header.stamp = self._stamp()
            pose.pose.position.x = s.x
            pose.pose.position.y = s.y
            pose.pose.position.z = s.z
            qx, qy, qz, qw = yaw_to_quaternion(s.yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info(f'/ground_truth_path 발행 ({len(path.poses)} poses)')

    def publish_markers(self) -> None:
        arr = MarkerArray()
        mid = 0

        for marker_id, (mx, my) in MARKERS.items():
            m = Marker()
            m.header.frame_id = FRAME_ID
            m.header.stamp = self._stamp()
            m.ns = 'aruco_markers'
            m.id = mid; mid += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = mx
            m.pose.position.y = my
            m.pose.position.z = 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 0.02
            m.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            arr.markers.append(m)

            text = Marker()
            text.header.frame_id = FRAME_ID
            text.header.stamp = self._stamp()
            text.ns = 'aruco_marker_labels'
            text.id = mid; mid += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = mx
            text.pose.position.y = my
            text.pose.position.z = 0.6
            text.pose.orientation.w = 1.0
            text.scale.z = 0.4
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"id{marker_id}"
            arr.markers.append(text)

            hover = Marker()
            hover.header.frame_id = FRAME_ID
            hover.header.stamp = self._stamp()
            hover.ns = 'hover_points'
            hover.id = mid; mid += 1
            hover.type = Marker.CYLINDER
            hover.action = Marker.ADD
            hover.pose.position.x = mx
            hover.pose.position.y = my
            hover.pose.position.z = ALTITUDE
            hover.pose.orientation.w = 1.0
            hover.scale.x = 0.6
            hover.scale.y = 0.6
            hover.scale.z = 0.05
            hover.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.6)
            arr.markers.append(hover)

        home = Marker()
        home.header.frame_id = FRAME_ID
        home.header.stamp = self._stamp()
        home.ns = 'home'
        home.id = mid; mid += 1
        home.type = Marker.SPHERE
        home.action = Marker.ADD
        home.pose.position.x = HOME[0]
        home.pose.position.y = HOME[1]
        home.pose.position.z = 0.1
        home.pose.orientation.w = 1.0
        home.scale.x = home.scale.y = home.scale.z = 0.4
        home.color = ColorRGBA(r=0.1, g=0.8, b=0.2, a=1.0)
        arr.markers.append(home)

        self.marker_pub.publish(arr)
        self.get_logger().info(f'/ground_truth_markers 발행 ({len(arr.markers)} markers)')

    def republish(self) -> None:
        self.publish_path()
        self.publish_markers()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
