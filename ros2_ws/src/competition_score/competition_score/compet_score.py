#!/usr/bin/env python3

import csv
import json
import math
from pathlib import Path

import rclpy
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class OfflineScorer(Node):

    def __init__(self):
        super().__init__('competition_score')

        self.declare_parameter('team', 'team_alpha')
        # competition_gui 가 만드는 위치와 같은 곳에 둔다(그쪽도
        # Path.home()/'competition_results'). 원래는 특정 PC의 절대경로가
        # 박혀 있어 다른 PC에서는 저장이 실패했다.
        self.declare_parameter(
            'output_dir', str(Path.home() / 'competition_results'))
        self.declare_parameter('pose_topic', '/drone/estimated_pose')
        self.declare_parameter('event_topic', '/competition/mission_event')

        # 구획선 교차점. 규정집은 "격자 크기 약 4m x 4m"인데 30m는 4로
        # 나눠떨어지지 않아(7.5칸) X는 7칸 4.2857m, Y는 20m라 정확히 5칸 4.0m로
        # 잡았다. gazebo/worlds/generate_competition_map.py 의 x_positions /
        # y_positions 와 동일해야 한다 — 라인트레이싱 이탈 오차를 이 좌표
        # 기준으로 재므로, 5m 등간격으로 두면 격자선을 정확히 따라가도
        # 오차가 있는 것처럼 채점된다.
        self.declare_parameter(
            'grid_x', [0.0, 4.2857, 8.5714, 12.8571, 17.1429, 21.4286,
                       25.7143, 30.0])
        self.declare_parameter('grid_y', [0.0, 4.0, 8.0, 12.0, 16.0, 20.0])

        self.declare_parameter('home', [0., 0.])
        self.declare_parameter('marker_1', [4.2857, 4.0])
        self.declare_parameter('marker_2', [4.2857, 16.0])
        self.declare_parameter('marker_3', [25.7143, 4.0])
        self.declare_parameter('marker_4', [12.8571, 12.0])

        self.declare_parameter('target_time', 180.0)
        self.declare_parameter('maximum_time', 600.0)

        self.team = self.get_parameter('team').value
        self.output_dir = Path(self.get_parameter('output_dir').value)

        self.grid_x = self.get_parameter('grid_x').value
        self.grid_y = self.get_parameter('grid_y').value
        self.home = self.get_parameter('home').value

        self.markers = {
            i: self.get_parameter(f'marker_{i}').value
            for i in range(1, 5)
        }

        self.target_time = self.get_parameter('target_time').value
        self.maximum_time = self.get_parameter('maximum_time').value

        self.state = 'IDLE'
        self.marker = 0
        self.segment = None
        self.started = False
        self.start_time = None

        self.rows = []
        self.grid_errors = []
        self.altitude_errors = []
        self.hover_errors = {i: [] for i in range(1, 5)}
        self.segment_errors = {'1-2': [], '2-3': [], '3-4': [], '4-0': []}
        self.detected = set()
        self.valid_hover = set()
        self.hover_start = {}

        # self.create_subscription(
        #     PoseStamped,
        #     self.get_parameter('pose_topic').value,
        #     self.pose_callback,
        #     qos_profile_sensor_data
        # )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('pose_topic').value,
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            String,
            self.get_parameter('event_topic').value,
            self.event_callback,
            20
        )

        self.get_logger().info('Offline scorer ready for rosbag replay')

    @staticmethod
    def rmse(values):
        return math.sqrt(sum(v * v for v in values) / len(values)) \
            if values else float('inf')

    @staticmethod
    def score(error, limit):
        return max(0.0, 100.0 * (1.0 - error / limit)) \
            if math.isfinite(error) else 0.0

    @staticmethod
    def point_segment_distance(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        d2 = dx * dx + dy * dy

        if d2 == 0.0:
            return math.hypot(p[0] - a[0], p[1] - a[1])

        u = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / d2
        u = max(0.0, min(1.0, u))

        qx, qy = a[0] + u * dx, a[1] + u * dy
        return math.hypot(p[0] - qx, p[1] - qy)

    def elapsed(self, msg=None):
        if msg and (msg.header.stamp.sec or msg.header.stamp.nanosec):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            t = self.get_clock().now().nanoseconds * 1e-9

        return t - self.start_time

    def pose_callback(self, msg):
        if not self.started:
            return

        # x = msg.pose.position.x
        # y = msg.pose.position.y
        # z = msg.pose.position.z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        t = self.elapsed(msg)

        grid_error = ''
        hover_error = ''
        segment_error = ''

        self.altitude_errors.append(abs(z - 2.0))

        if self.state == 'SEARCH':
            grid_error = min(
                min(abs(x - gx) for gx in self.grid_x),
                min(abs(y - gy) for gy in self.grid_y)
            )
            self.grid_errors.append(grid_error)

        elif self.state == 'HOVER' and self.marker:
            mx, my = self.markers[self.marker]
            hover_error = math.hypot(x - mx, y - my)
            self.hover_errors[self.marker].append(hover_error)

        elif self.state in ('TRACE', 'RETURN') and self.segment:
            a_id, b_id = self.segment
            a = self.markers[a_id]
            b = self.home if b_id == 0 else self.markers[b_id]

            segment_error = self.point_segment_distance((x, y), a, b)
            self.segment_errors[f'{a_id}-{b_id}'].append(segment_error)

        self.rows.append([
            t, x, y, z, self.state, self.marker,
            grid_error, hover_error, segment_error
        ])

    def event_callback(self, msg):
        event = msg.data.strip()
        now = self.get_clock().now().nanoseconds * 1e-9

        if event == 'START':
            self.started = True
            self.start_time = now
            self.state = 'TAKEOFF'
            return

        if not self.started:
            return

        parts = event.split(':')

        if event in ('SEARCH_START', 'SEARCH_RESUME'):
            self.state = 'SEARCH'
            self.marker = 0

        elif parts[0] == 'MARKER_DETECTED':
            self.detected.add(int(parts[1]))

        elif parts[0] == 'HOVER_START':
            self.marker = int(parts[1])
            self.hover_start[self.marker] = now
            self.hover_errors[self.marker].clear()
            self.state = 'HOVER'

        elif parts[0] == 'HOVER_END':
            marker = int(parts[1])
            duration = now - self.hover_start.get(marker, now)

            if duration >= 5.0 and self.rmse(self.hover_errors[marker]) <= 0.30:
                self.valid_hover.add(marker)

            self.state = 'SEARCH'
            self.marker = 0

        elif parts[0] == 'TRACE_START':
            self.segment = (int(parts[1]), int(parts[2]))
            self.state = 'TRACE'

        elif event == 'RETURN_HOME':
            self.segment = (4, 0)
            self.state = 'RETURN'

        elif event in ('FINISH', 'ABORT'):
            self.finish(aborted=event == 'ABORT')

    def finish(self, aborted=False):
        completion_time = self.get_clock().now().nanoseconds * 1e-9 - self.start_time

        grid_rmse = self.rmse(self.grid_errors)
        altitude_rmse = self.rmse(self.altitude_errors)

        all_segment_errors = [
            value
            for values in self.segment_errors.values()
            for value in values
        ]
        segment_rmse = self.rmse(all_segment_errors)

        hover_scores = [
            self.score(self.rmse(self.hover_errors[i]), 0.50)
            if i in self.valid_hover else 0.0
            for i in range(1, 5)
        ]

        time_score = 100.0 if completion_time <= self.target_time else max(
            0.0,
            100.0 * (self.maximum_time - completion_time) /
            (self.maximum_time - self.target_time)
        )

        scores = {
            'grid': self.score(grid_rmse, 0.75),
            'hover': sum(hover_scores) / 4.0,
            'segments': self.score(segment_rmse, 0.75),
            'altitude': self.score(altitude_rmse, 0.40),
            'time': time_score
        }

        penalty = 10.0 * (4 - len(self.detected))
        penalty += 5.0 * (len(self.detected - self.valid_hover))

        final = (
            0.25 * scores['grid'] +
            0.20 * scores['hover'] +
            0.25 * scores['segments'] +
            0.15 * scores['altitude'] +
            0.15 * scores['time'] -
            penalty
        )

        if aborted:
            final = 0.0

        result = {
            'team': self.team,
            'completion_time_s': completion_time,
            'detected_markers': sorted(self.detected),
            'valid_hover_markers': sorted(self.valid_hover),
            'grid_rmse_m': grid_rmse,
            'segment_rmse_m': segment_rmse,
            'altitude_rmse_m': altitude_rmse,
            'scores': scores,
            'penalty': penalty,
            'final_score': max(0.0, min(100.0, final))
        }

        folder = self.output_dir / self.team
        folder.mkdir(parents=True, exist_ok=True)

        with open(folder / 'trajectory.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'time', 'x', 'y', 'z', 'state', 'marker',
                'grid_error', 'hover_error', 'segment_error'
            ])
            writer.writerows(self.rows)

        with open(folder / 'result.json', 'w') as file:
            json.dump(result, file, indent=2)

        self.get_logger().info(
            f'Result saved: {folder / "result.json"}'
        )

        self.started = False


def main():
    rclpy.init()
    node = OfflineScorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()