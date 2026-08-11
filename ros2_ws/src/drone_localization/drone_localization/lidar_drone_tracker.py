#!/usr/bin/env python3
"""
Two-LiDAR drone 3D localization node (outside-in, ground-fixed sensors).

Two ground LiDARs (east + north, perpendicular views) each publish a
PointCloud2. Each callback transforms its cloud into the shared Gazebo
world frame and buffers it. A fixed-rate timer then fuses the buffers and
runs one pipeline + KF step:

  /lidar1/points ─┐ (→ world frame, buffered)
                  ├─ concat ─→ ROI filter (z-band = hover altitude only)
  /lidar2/points ─┘             → voxel downsampling (open3d)
                                 → statistical outlier removal (rain/scatter)
                                 → DBSCAN clustering (open3d, PCL-equivalent)
                                 → drone cluster selection
                                 → KF predict+update (6-state const-velocity)
                                 → PoseWithCovarianceStamped /drone/estimated_pose

Ground-state (pre-liftoff/post-landing) tracking is intentionally out of
scope: the ROI z-band only spans the hover altitude, so the pipeline never
sees the floor and needs no separate ground-removal step.

Fusing two perpendicular views triangulates the target in 3D and roughly
doubles the number of hits on the small (~50cm) drone.

All spatial operations are in the Gazebo world frame (ENU, Z-up).
Field spans X:[0,30] Y:[0,20]; field center = (15, 10).
Each LiDAR must be outside the 33x23m exclusion zone around field center.
Sensor poses MUST match the LIDARS list in generate_competition_map.py.

Dependencies (install once):
    pip3 install open3d scipy
"""

import csv
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy, QoSProfile, ReliabilityPolicy
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    raise ImportError('scipy가 설치되지 않았습니다. pip3 install scipy 실행하세요.')

try:
    import open3d as o3d
except ImportError:
    raise ImportError('open3d가 설치되지 않았습니다. pip3 install open3d 실행하세요.')

# Use sensor_msgs_py if available (ROS2 Humble ships it), fall back to sensor_msgs
try:
    from sensor_msgs_py import point_cloud2 as pc2_util
except ImportError:
    from sensor_msgs import point_cloud2 as pc2_util


# ---------------------------------------------------------------------------
# KF: 6-state (x, y, z, vx, vy, vz), constant-velocity model (standard KF —
# F and H are both linear/constant, so there is nothing "extended" here)
# ---------------------------------------------------------------------------
class KF6D:
    def __init__(self, proc_noise_pos: float, proc_noise_vel: float, meas_noise_pos: float):
        self.x = np.zeros(6)
        self.P = np.eye(6) * 9.0  # high initial uncertainty = 3m std

        # Process noise: position changes by ≤ proc_noise_pos per frame,
        # velocity changes by ≤ proc_noise_vel per frame at 10Hz
        self.Q = np.diag([
            proc_noise_pos**2, proc_noise_pos**2, proc_noise_pos**2,
            proc_noise_vel**2, proc_noise_vel**2, proc_noise_vel**2,
        ])

        # Measurement noise: raw LiDAR range noise is 0.8cm, but centroid
        # estimation from a sparse side-view cluster of a 50cm drone adds
        # 5–15cm uncertainty (fewer points → larger centroid variance).
        # Using 15cm std is a conservative but realistic choice.
        self.R = np.eye(3) * meas_noise_pos**2

        # Observation matrix: only position is measured
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.initialized = False

    def init(self, pos: np.ndarray):
        self.x[:3] = pos
        self.x[3:] = 0.0
        self.P = np.eye(6) * 0.25  # 0.5m initial uncertainty after first detection
        self.initialized = True

    def predict(self, dt: float):
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z: np.ndarray):
        y = z - self.H @ self.x                         # innovation
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.solve(S, np.eye(3))  # Kalman gain
        self.x = self.x + K @ y
        # Joseph form: guarantees P stays symmetric/positive-definite even
        # under floating-point roundoff (the (I-KH)@P form can lose that).
        IKH = np.eye(6) - K @ self.H
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T

    @property
    def position(self) -> np.ndarray:
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:].copy()

    @property
    def cov_pos(self) -> np.ndarray:
        return self.P[:3, :3].copy()


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------
class SensorCfg:
    """One ground-fixed LiDAR: its topic and sensor→world transform."""

    def __init__(self, name, topic, x, y, z, roll, pitch, yaw):
        self.name = name
        self.topic = topic
        self.x, self.y, self.z = x, y, z
        self.roll, self.pitch, self.yaw = roll, pitch, yaw
        self.R_s2w = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        self.t_s2w = np.array([x, y, z])

    def to_world(self, pts_sensor: np.ndarray) -> np.ndarray:
        return (self.R_s2w @ pts_sensor.T).T + self.t_s2w


class LidarDroneTracker(Node):
    def __init__(self):
        super().__init__('lidar_drone_tracker')

        self._declare_parameters()
        self._read_parameters()
        self._build_sensors()          # builds all sensor configs

        self._kf = KF6D(
            self.proc_noise_pos, self.proc_noise_vel, self.meas_noise_pos)
        # Seed the filter at the known home pad so tracking is locked from the
        # ground and the selection prefers the cluster nearest to the pad.
        if self.seed_at_home:
            self._kf.init(self.home_xyz.copy())
        self._coast_count = 0

        # 클러스터 모양 덤프 (cluster_dump_path 가 설정된 경우에만 수집)
        self._dump = [] if self.cluster_dump_path else None
        self._last_sel_pts = None
        self._last_sel_mask = None

        # Waypoint (마커) 호버 감지 상태
        self._wp_active_id = None
        self._wp_samples = []
        self._wp_break_count = 0
        self._wp_visit_counts = {}
        self._wp_markers = MarkerArray()

        os.makedirs(self.waypoint_log_dir, exist_ok=True)
        self._wp_csv_path = os.path.join(
            self.waypoint_log_dir, 'waypoint_estimates.csv')
        self._wp_csv_f = open(self._wp_csv_path, 'w', newline='')
        self._wp_csv_w = csv.writer(self._wp_csv_f)
        self._wp_csv_w.writerow(
            ['marker_id', 'visit', 'x', 'y', 'z', 'n_samples', 'duration_s'])

        # Latest world-frame cloud per sensor: index → (Nx3 array, monotonic_t)
        self._buffers: list = [None] * len(self._sensors)

        # Detection-health stats, logged every _stat_period seconds
        self._stat_period = 2.0
        self._stat_t0 = time.monotonic()
        self._stat_frames = 0          # processing ticks with any input cloud
        self._stat_detect = 0          # ticks where a drone cluster was picked
        self._stat_cluster_pts = 0     # sum of selected-cluster sizes
        self._stat_cand = 0            # sum of candidate-cluster counts
        self._stat_merged_pts = 0      # sum of fused raw point counts
        n = len(self._sensors)
        self._stat_sensor_pts = [0] * n   # per-sensor points contributed
        self._stat_sensor_hits = [0] * n  # per-sensor frames contributed

        # 검출 실패를 파이프라인 단계별로 분류한다. "몇 % 놓쳤다"만 알면
        # 어디를 고쳐야 할지 알 수 없어서, 탈락 지점을 직접 센다.
        #   roi     ROI 통과 점이 10개 미만
        #   sparse  DBSCAN 돌리기엔 점이 부족
        #   noclust DBSCAN이 클러스터를 못 만듦
        #   size    클러스터는 생겼지만 크기 필터에서 전부 탈락
        #   gate    후보는 있었지만 연관 게이트가 거부
        self._stat_fail = dict(roi=0, sparse=0, noclust=0, size=0, gate=0)
        self._stat_rej_small = 0   # 크기필터: min_pts 미만이라 버린 클러스터
        self._stat_rej_big = 0     # 크기필터: max_pts 초과라 버린 클러스터

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # One subscription per LiDAR; each stores its cloud in world frame.
        self._subs = [
            self.create_subscription(
                PointCloud2, self._sensors[0].topic, self._cb_lidar1, sensor_qos),
            self.create_subscription(
                PointCloud2, self._sensors[1].topic, self._cb_lidar2, sensor_qos),
        ]

        self._pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/drone/estimated_pose', reliable_qos)
        self._pub_filtered = self.create_publisher(
            PointCloud2, '/lidar/filtered_points', sensor_qos)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/lidar/cluster_markers', reliable_qos)
        self._pub_waypoints = self.create_publisher(
            MarkerArray, '/drone/waypoint_estimates', reliable_qos)

        # Single processing clock at the LiDAR rate: fuse both buffers,
        # run the pipeline, and step the KF at a fixed dt. Decoupling the
        # fusion from the two async callbacks keeps KF timing consistent.
        self._proc_timer = self.create_timer(self._dt, self._process)

        names = ', '.join(
            f'{c.name}({c.topic} @ {c.x:.1f},{c.y:.1f},{c.z:.1f})'
            for c in self._sensors)
        self.get_logger().info(
            f'LidarDroneTracker ready with {len(self._sensors)} LiDAR(s): '
            f'{names}. Fusing at {self.update_rate:.0f} Hz.')

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        # --- LiDAR 1 pose in Gazebo world frame (ENU) ---
        # Default: east side, x=35 (outside the 31.5m exclusion boundary),
        # facing west (yaw=π) toward field centre (15,10).
        self.declare_parameter('lidar1_topic', '/lidar1/points')
        self.declare_parameter('lidar1_x',     35.0)
        self.declare_parameter('lidar1_y',     10.0)
        self.declare_parameter('lidar1_z',      2.5)
        self.declare_parameter('lidar1_roll',   0.0)
        self.declare_parameter('lidar1_pitch',  0.0)
        self.declare_parameter('lidar1_yaw',    math.pi)

        # --- LiDAR 2 pose (perpendicular view for triangulation) ---
        # Default: north side, y=25 (outside 21.5m boundary), facing south.
        self.declare_parameter('lidar2_topic', '/lidar2/points')
        self.declare_parameter('lidar2_x',     15.0)
        self.declare_parameter('lidar2_y',     25.0)
        self.declare_parameter('lidar2_z',      2.5)
        self.declare_parameter('lidar2_roll',   0.0)
        self.declare_parameter('lidar2_pitch',  0.0)
        self.declare_parameter('lidar2_yaw',   -math.pi / 2)

        # Max age (s) of a sensor's buffered cloud before it's ignored in
        # fusion (2× the frame period tolerates one dropped frame).
        self.declare_parameter('buffer_stale_timeout', 0.25)

        self.declare_parameter('voxel_leaf_size', 0.05)

        # ROI in world frame. X/Y: 2m margin around the 30×20 field.
        # Z: hover-altitude band only (1.0–3.0m) — this is the ONLY height
        # filter in the pipeline now. Ground-state (pre-liftoff/post-landing)
        # tracking is out of scope, so points near the floor never need to be
        # separated from the drone; the ROI simply never lets them through.
        self.declare_parameter('roi_x_min', -2.0)
        self.declare_parameter('roi_x_max', 32.0)
        self.declare_parameter('roi_y_min', -2.0)
        self.declare_parameter('roi_y_max', 22.0)
        self.declare_parameter('roi_z_min',  1.0)
        self.declare_parameter('roi_z_max',  3.0)

        # Statistical outlier removal (rain / scatter noise)
        self.declare_parameter('sor_mean_k',     20)
        self.declare_parameter('sor_std_ratio',   2.0)

        # DBSCAN clustering (PCL EuclideanClusterExtraction equivalent)
        # eps=25cm: connects points within the same drone body (50cm–1m)
        # min_pts=3: side-view LiDAR gives few hits on a small drone at distance
        self.declare_parameter('cluster_tolerance',  0.25)
        self.declare_parameter('cluster_min_points',  3)
        self.declare_parameter('cluster_max_points',  400)

        # Home-seed: seeds the KF near where the drone should first appear so
        # early-frame candidate selection (nearest-to-track) has a prior
        # instead of falling back to cold-start (largest cluster). The ROI
        # only sees the hover band, so home_z must be a hover altitude
        # (~2.0m) — NOT the pad height, which now sits outside the ROI
        # entirely and would just fail the association gate on first detection.
        self.declare_parameter('seed_at_home', True)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', 2.0)

        # KF noise
        self.declare_parameter('proc_noise_pos',  0.05)
        self.declare_parameter('proc_noise_vel',  0.30)
        self.declare_parameter('meas_noise_pos',  0.15)

        # Gating: reject a candidate farther than this from the KF prediction.
        # This is the BASE (well-tracked) radius in metres; the effective gate
        # widens with the filter's own uncertainty — see _select_drone_cluster.
        self.declare_parameter('max_assoc_dist', 3.0)
        # How many sigma of position uncertainty to add to the base radius.
        self.declare_parameter('gate_sigma_k', 3.0)
        # Hard ceiling on the widened gate, so a long dropout can never open it
        # far enough to latch onto unrelated clutter.
        self.declare_parameter('max_assoc_dist_cap', 4.0)

        # Coasting: max frames to continue pure-predict when no detection
        self.declare_parameter('coast_max_frames', 5)
        # Prolonged loss: after this many coast frames, drop the KF entirely
        # (cold-start) so the next detection is free to re-lock on the
        # largest cluster instead of being gated against a stale prediction.
        self.declare_parameter('reset_after_frames', 50)  # 5초 @ 10Hz

        self.declare_parameter('lidar_update_rate', 10.0)

        # ---------------------------------------------------------------
        # Waypoint(마커) 호버 감지 + 위치 재추정
        #
        # 대회 규정에 마커 위 호버 시간이 정해져 있지 않고 팀마다 체류
        # 패턴이 다르므로, 고정 시간이 아니라 "느림 + 근접" 조건으로
        # 판정한다. 채점기준 "경로점 위치추정" 하 등급이 <4m이므로,
        # 게이트 반경은 이보다 여유있게 잡아 항법이 부정확한 팀도
        # 표본을 놓치지 않게 한다 (좁게 잡으면 그런 팀은 표본이 아예
        # 안 잡혀서 채점 자체가 불가능해짐).
        # 선택된 드론 클러스터의 원본 점들을 파일로 남긴다(비었으면 끔).
        # 클러스터가 실제로 어떤 모양·분포로 잡히는지 사후에 확인하는 용도.
        self.declare_parameter('cluster_dump_path', '')

        self.declare_parameter('waypoint_gate_radius', 4.5)       # m
        self.declare_parameter('waypoint_speed_threshold', 0.25)  # m/s
        # 최소 연속 프레임(4프레임 @ 10Hz ≈ 0.4초). 이보다 짧으면 노이즈로
        # 버림 — 아주 잠깐 스치듯 지나가는 팀도 놓치지 않게 여유있게(짧게) 잡음.
        self.declare_parameter('waypoint_min_frames', 4)
        # 짧은 이탈 유예 프레임(5프레임 @ 10Hz = 0.5초). 속도추정이 정지
        # 중에도 미세하게 흔들려 문턱값을 순간적으로 넘나드는 것 때문에
        # 하나의 호버가 여러 조각으로 쪼개지는 걸 방지 — 이 프레임 수
        # 이내로 짧게 이탈하면 무시하고 같은 방문으로 이어붙인다.
        self.declare_parameter('waypoint_grace_frames', 5)
        self.declare_parameter(
            'waypoint_log_dir', os.path.expanduser('~/drone_project/flight_logs'))

        # 마커 좌표. generate_competition_map.py의 marker_coords와 항상
        # 동일하게 유지할 것 (LIDARS 좌표와 같은 방식으로 수동 동기화).
        self.declare_parameter('marker1_x', 4.2857)
        self.declare_parameter('marker1_y', 4.0)
        self.declare_parameter('marker2_x', 4.2857)
        self.declare_parameter('marker2_y', 16.0)
        self.declare_parameter('marker3_x', 25.7143)
        self.declare_parameter('marker3_y', 4.0)
        self.declare_parameter('marker4_x', 12.8571)
        self.declare_parameter('marker4_y', 12.0)

    def _read_parameters(self):
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.buffer_stale_timeout = gp('buffer_stale_timeout')
        self.voxel_leaf_size = gp('voxel_leaf_size')
        self.roi_x_min = gp('roi_x_min')
        self.roi_x_max = gp('roi_x_max')
        self.roi_y_min = gp('roi_y_min')
        self.roi_y_max = gp('roi_y_max')
        self.roi_z_min = gp('roi_z_min')
        self.roi_z_max = gp('roi_z_max')
        self.sor_k        = gp('sor_mean_k')
        self.sor_std      = gp('sor_std_ratio')
        self.eps          = gp('cluster_tolerance')
        self.min_pts      = gp('cluster_min_points')
        self.max_pts      = gp('cluster_max_points')
        self.seed_at_home = gp('seed_at_home')
        self.home_xyz     = np.array(
            [gp('home_x'), gp('home_y'), gp('home_z')])
        self.max_assoc_dist = gp('max_assoc_dist')
        self.gate_sigma_k = gp('gate_sigma_k')
        self.max_assoc_dist_cap = gp('max_assoc_dist_cap')
        self.proc_noise_pos = gp('proc_noise_pos')
        self.proc_noise_vel = gp('proc_noise_vel')
        self.meas_noise_pos = gp('meas_noise_pos')
        self.cluster_dump_path = gp('cluster_dump_path')
        self.waypoint_gate_radius = gp('waypoint_gate_radius')
        self.waypoint_speed_threshold = gp('waypoint_speed_threshold')
        self.waypoint_min_frames = gp('waypoint_min_frames')
        self.waypoint_grace_frames = gp('waypoint_grace_frames')
        self.waypoint_log_dir = gp('waypoint_log_dir')
        self._marker_xy = {
            1: (gp('marker1_x'), gp('marker1_y')),
            2: (gp('marker2_x'), gp('marker2_y')),
            3: (gp('marker3_x'), gp('marker3_y')),
            4: (gp('marker4_x'), gp('marker4_y')),
        }
        self.coast_max    = gp('coast_max_frames')
        self.reset_after  = gp('reset_after_frames')
        self.update_rate  = gp('lidar_update_rate')
        self._dt = 1.0 / self.update_rate

    def _build_sensors(self):
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        specs = [
            ('lidar1', gp('lidar1_topic'), gp('lidar1_x'), gp('lidar1_y'),
             gp('lidar1_z'), gp('lidar1_roll'), gp('lidar1_pitch'),
             gp('lidar1_yaw')),
            ('lidar2', gp('lidar2_topic'), gp('lidar2_x'), gp('lidar2_y'),
             gp('lidar2_z'), gp('lidar2_roll'), gp('lidar2_pitch'),
             gp('lidar2_yaw')),
        ]

        self._sensors = []
        for name, topic, x, y, z, roll, pitch, yaw in specs:
            self._sensors.append(SensorCfg(name, topic, x, y, z, roll, pitch, yaw))

    # ------------------------------------------------------------------
    # Per-sensor callback: store the latest cloud in WORLD frame
    # ------------------------------------------------------------------
    def _cb_lidar1(self, msg: PointCloud2):
        pts = self._pc2_to_numpy(msg)
        if pts is None or len(pts) == 0:
            return
        world = self._sensors[0].to_world(pts)
        self._buffers[0] = (world, time.monotonic(), msg.header.stamp)

    def _cb_lidar2(self, msg: PointCloud2):
        pts = self._pc2_to_numpy(msg)
        if pts is None or len(pts) == 0:
            return
        world = self._sensors[1].to_world(pts)
        self._buffers[1] = (world, time.monotonic(), msg.header.stamp)

    # ------------------------------------------------------------------
    # Fusion + pipeline + KF, driven by a fixed-rate timer
    # ------------------------------------------------------------------
    def _process(self):
        now = time.monotonic()
        valid = []
        for i, buf in enumerate(self._buffers):
            if buf is None:
                continue
            pts, t, stamp = buf
            if now - t <= self.buffer_stale_timeout:
                valid.append((i, pts, stamp))
                self._stat_sensor_pts[i] += len(pts)
                self._stat_sensor_hits[i] += 1

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        if len(valid) == 0:
            self._coast(header)
        elif len(valid) == 2:
            # 두 LiDAR stamp 차이 확인
            t0 = valid[0][2].sec + valid[0][2].nanosec * 1e-9
            t1 = valid[1][2].sec + valid[1][2].nanosec * 1e-9
            if abs(t0 - t1) < 0.05:  # 50ms 이내만 융합
                pts_w = np.concatenate([valid[0][1], valid[1][1]], axis=0)
                # 합치고 나면 어느 점이 어느 센서에서 왔는지 알 수 없다.
                # 융합 방식(밀도가중 vs 동일가중)을 사후 비교하려면 출처가
                # 필요하므로, 이어붙이는 순서 그대로 센서 인덱스를 남긴다.
                src = np.concatenate([
                    np.full(len(valid[0][1]), valid[0][0], dtype=np.int8),
                    np.full(len(valid[1][1]), valid[1][0], dtype=np.int8)])
                self._process_cloud(pts_w, header, src)
            else:
                # 시간차 너무 크면 최신 것만 사용
                k = 0 if t0 > t1 else 1
                self._process_cloud(
                    valid[k][1], header,
                    np.full(len(valid[k][1]), valid[k][0], dtype=np.int8))
        else:
            # LiDAR 1개만 유효한 경우
            self._process_cloud(
                valid[0][1], header,
                np.full(len(valid[0][1]), valid[0][0], dtype=np.int8))

        self._maybe_log_stats(now)

    def _maybe_log_stats(self, now: float):
        if now - self._stat_t0 < self._stat_period:
            return
        f = self._stat_frames
        if f == 0:
            self.get_logger().warn(
                'No LiDAR input in the last '
                f'{self._stat_period:.0f}s — check /lidar1,2/points and Agent.')
        else:
            det = self._stat_detect
            rate = 100.0 * det / f
            avg_pts = (self._stat_cluster_pts / det) if det else 0.0
            avg_cand = self._stat_cand / f
            # Per-sensor fusion breakdown: how often each LiDAR contributed
            # and how many points on average → confirms both are fused.
            sensors = []
            for i, cfg in enumerate(self._sensors):
                hits = self._stat_sensor_hits[i]
                pct = 100.0 * hits / f
                avg = (self._stat_sensor_pts[i] / hits) if hits else 0.0
                sensors.append(f'{cfg.name}:{avg:.0f}점({pct:.0f}%)')
            self.get_logger().info(
                f'[검출] {det}/{f} 프레임 ({rate:.0f}%)  '
                f'선택 클러스터 평균 {avg_pts:.1f}점  '
                f'후보 {avg_cand:.1f}개  '
                f'융합[{" + ".join(sensors)}]')

            # 놓친 프레임이 있으면 어느 단계에서 빠졌는지 함께 보고한다.
            # 이게 없으면 "몇 % 놓쳤다"만 알 뿐 어디를 고쳐야 할지 모른다.
            miss = f - det
            if miss > 0:
                LBL = {'roi': 'ROI통과<10점', 'sparse': '점부족',
                       'noclust': '클러스터0', 'size': '크기필터', 'gate': '게이트'}
                parts = [f'{LBL[k]} {v}' for k, v in self._stat_fail.items() if v]
                rej = []
                if self._stat_rej_small:
                    rej.append(f'작음(<{self.min_pts}점) {self._stat_rej_small}')
                if self._stat_rej_big:
                    rej.append(f'큼(>{self.max_pts}점) {self._stat_rej_big}')
                self.get_logger().warn(
                    f'  └ 미검출 {miss}프레임 원인: '
                    + (', '.join(parts) if parts else '분류안됨')
                    + (f'   [버려진 클러스터: {", ".join(rej)}]' if rej else ''))
        self._stat_t0 = now
        self._stat_frames = self._stat_detect = 0
        self._stat_cluster_pts = self._stat_cand = self._stat_merged_pts = 0
        n = len(self._sensors)
        self._stat_sensor_pts = [0] * n
        self._stat_sensor_hits = [0] * n
        self._stat_fail = dict.fromkeys(self._stat_fail, 0)
        self._stat_rej_small = self._stat_rej_big = 0

    def _process_cloud(self, pts_w: np.ndarray, header: Header,
                       src: np.ndarray | None = None):
        """Run the full pipeline on the fused world-frame cloud, step KF.

        src는 점별 출처 센서 인덱스(0/1). 클러스터 덤프에만 쓰이며 파이프라인
        동작에는 영향이 없다. 점을 걸러낼 때마다 같은 마스크를 태그에도
        적용해 순서를 맞춰야 한다."""
        self._stat_frames += 1
        self._stat_merged_pts += len(pts_w)
        # ROI filter in world frame (drops inf/nan-origin points too, since
        # any comparison with them is False)
        mask = (
            (pts_w[:, 0] >= self.roi_x_min) & (pts_w[:, 0] <= self.roi_x_max) &
            (pts_w[:, 1] >= self.roi_y_min) & (pts_w[:, 1] <= self.roi_y_max) &
            (pts_w[:, 2] >= self.roi_z_min) & (pts_w[:, 2] <= self.roi_z_max)
        )
        pts_w = pts_w[mask]
        if src is not None:
            src = src[mask]
        # 예전엔 여기가 하드코딩 10이었다. ROI가 z 0~6m로 넓어 바닥까지
        # 들어오던 시절에는 "10점 미만 = 센서가 아무것도 못 봄"이 맞았지만,
        # ROI를 호버 고도 띠(1~3m)로 좁힌 뒤로는 그 안에 드론밖에 없다.
        # 드론이 내놓는 점은 중앙값 14개, 원거리에선 3~7개라 문턱 10이
        # 정상 검출을 통째로 버리고 있었다(검출률 37%의 주범).
        # 군집화 최소 점수와 같은 기준으로 낮춘다 — 그 이하면 어차피
        # DBSCAN이 클러스터를 못 만든다.
        if len(pts_w) < self.min_pts:
            self._stat_fail['roi'] += 1
            self._coast(header)
            return

        # Open3D processing: voxel → outlier removal
        # (no separate ground-removal stage: the ROI z-band above already
        # excludes everything outside the hover altitude, floor included)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_w.astype(np.float64))

        # voxel downsampling 비활성화
        # 드론처럼 클러스터 포인트 수가 적은 상황에서 centroid 추정 정확도
        # 저하 우려 - 계산 속도와 정확도 트레이드오프 - 필요시 아래 주석 해제
        # pcd = pcd.voxel_down_sample(self.voxel_leaf_size)

        # SOR 비활성화
        # 비/안개 등 산란잡음 억제용인데 지금은 그런 잡음원이 없고, OS1-128의
        # 조밀한 클러스터(수십~수백 점)에서 뭘 걸러내는지 불확실해 디버깅 중
        # 변수를 줄이려고 끔 - 필요시 아래 주석 해제
        # if len(pcd.points) > max(self.sor_k * 5, 100):
        #     pcd, _ = pcd.remove_statistical_outlier(
        #         nb_neighbors=self.sor_k, std_ratio=self.sor_std)

        # Publish filtered cloud for RViz2 debugging
        pts_f = np.asarray(pcd.points, dtype=np.float32)
        if len(pts_f) > 0:
            self._pub_filtered.publish(self._numpy_to_pc2(pts_f, header))

        # DBSCAN clustering
        if len(pcd.points) < self.min_pts:
            self._stat_fail['sparse'] += 1
            self._coast(header)
            return

        labels = np.array(
            pcd.cluster_dbscan(
                eps=self.eps,
                min_points=self.min_pts,
                print_progress=False,
            )
        )

        # KF predict FIRST, then associate against that prediction, then
        # update (textbook predict → associate → update). Selecting before
        # predicting would gate candidates against the PREVIOUS frame's
        # posterior, i.e. a prior one frame stale — at 1.5 m/s that is 0.15 m
        # of the association budget given away for nothing.
        self._kf.predict(self._dt)

        # Select the drone cluster (gated against the fresh prediction)
        centroid = self._select_drone_cluster(pts_f, labels, header)
        self._stat_cand += self._last_n_cand

        if centroid is not None:
            self._stat_detect += 1
            self._stat_cluster_pts += self._last_sel_n
            if self._dump is not None and self._last_sel_pts is not None:
                # 프레임마다 (시각, 점들, 센트로이드, 출처센서)를 모아
                # close()에서 한 번에 저장.
                # src는 ROI까지만 통과한 상태라 pts_f와 길이·순서가 같다.
                # voxel/SOR을 다시 켜면 그 전제가 깨지므로 길이를 확인하고,
                # 어긋나면 태그를 포기한다(-1 = 출처 미상).
                if src is not None and len(src) == len(pts_f):
                    tags = src[self._last_sel_mask].astype(np.int8)
                else:
                    tags = np.full(self._last_sel_n, -1, dtype=np.int8)
                self._dump.append((
                    header.stamp.sec + header.stamp.nanosec * 1e-9,
                    self._last_sel_pts.astype(np.float32).copy(),
                    centroid.astype(np.float32).copy(),
                    tags))
            if not self._kf.initialized:
                self._kf.init(centroid)
            else:
                # 관측 반영은 별도 훅으로 뺀다. 파생 클래스가 센서별 순차
                # 업데이트 등 다른 융합 방식을 끼워넣을 수 있게 하는 자리다.
                self._apply_measurement(centroid, pts_f, src, header)
            self._coast_count = 0
            self._update_waypoint_tracking(centroid)
        else:
            self._coast_count += 1
            if self._coast_count > self.reset_after:
                self.get_logger().warn(
                    f'Tracking LOST {self._coast_count}프레임 → KF 리셋, 재획득 대기')
                self._kf.initialized = False
                self._coast_count = 0
                self._finalize_waypoint_episode()
                return
            elif self._coast_count > self.coast_max:
                self.get_logger().warn(
                    f'Tracking LOST: {self._coast_count} consecutive '
                    'missed frames.', throttle_duration_sec=2.0)
                return

        if self._kf.initialized:
            self._publish_pose(header)

    def _apply_measurement(self, centroid, pts_f, src, header):
        """선택된 클러스터를 KF에 반영. 기본 구현은 융합 센트로이드 1회 갱신.

        pts_f/src는 파생 클래스가 센서별로 나눠 쓰라고 넘긴다(기본은 미사용).
        `self._last_sel_mask`로 선택 클러스터에 해당하는 부분을 고를 수 있다."""
        self._kf.update(centroid)

    # ------------------------------------------------------------------
    # Drone cluster selection
    # ------------------------------------------------------------------
    def _select_drone_cluster(
        self,
        pts: np.ndarray,
        labels: np.ndarray,
        header: Header,
    ) -> np.ndarray | None:
        self._last_n_cand = 0
        self._last_sel_n = 0
        unique = [l for l in np.unique(labels) if l >= 0]
        if not unique:
            self._stat_fail['noclust'] += 1
            return None

        candidates = []
        marker_msgs = MarkerArray()

        for label in unique:
            mask = labels == label
            cluster = pts[mask]
            n = len(cluster)
            if n < self.min_pts:
                self._stat_rej_small += 1
                continue
            if n > self.max_pts:
                self._stat_rej_big += 1
                continue

            # median, not mean: a stray outlier point in the cluster pulls
            # the mean noticeably but barely moves the median.
            centroid = np.median(cluster, axis=0)
            # No height gate here: the ROI z-band (roi_z_min/max) already
            # constrains every surviving point to the hover band, so any
            # cluster's centroid is guaranteed to already be inside it.
            candidates.append((label, centroid, n))

            # Debug sphere marker (green) at cluster centroid
            m = Marker()
            m.header = header
            m.header.frame_id = 'map'
            m.ns = 'lidar_clusters'
            m.id = int(label)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(centroid[0])
            m.pose.position.y = float(centroid[1])
            m.pose.position.z = float(centroid[2])
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.3
            m.color.a = 0.8
            m.lifetime.sec = 1
            marker_msgs.markers.append(m)

        self._pub_markers.publish(marker_msgs)

        self._last_n_cand = len(candidates)
        if not candidates:
            self._stat_fail['size'] += 1
            return None

        if self._kf.initialized:
            # Prefer the cluster nearest to the KF's PREDICTED position for
            # this frame (predict() already ran in _process_cloud), so the
            # prior accounts for how far the drone moved since last frame.
            # Seeded near the hover band at startup so the first frames have
            # a prior instead of falling back to cold-start. Still valid
            # while coasting — pure prediction remains a usable prior.
            kf_pos = self._kf.position
            candidates.sort(key=lambda c: float(np.linalg.norm(c[1] - kf_pos)))
            # Gate: reject the nearest candidate if it is implausibly far from
            # the prediction.
            #
            # The threshold is NOT fixed. A constant gate creates a vicious
            # cycle: once the prediction drifts past it (e.g. the constant-
            # velocity model overshoots while the drone decelerates), the real
            # drone is rejected, which starves the filter of corrections, which
            # makes it drift further — the track never recovers until the
            # whole filter is reset seconds later. Measured on 5 flights:
            # dropouts began at a median 1.13m error against a 1.5m gate, each
            # costing ~3.8s of silence.
            #
            # Instead scale it by the filter's own position uncertainty, which
            # already grows on every predict-without-update. Tight while
            # tracking well (so clutter is still rejected), wide right after a
            # miss (so re-acquisition is possible). Capped so it can never open
            # far enough to latch onto arbitrary clutter. This assumes nothing
            # about how the drone is flown, so it holds for any team's flight
            # style — unlike tuning the motion model for our own test mission.
            sigma = float(np.sqrt(np.trace(self._kf.cov_pos[:2, :2]) / 2.0))
            gate = min(self.max_assoc_dist + self.gate_sigma_k * sigma,
                       self.max_assoc_dist_cap)
            if float(np.linalg.norm(candidates[0][1] - kf_pos)) > gate:
                self._stat_fail['gate'] += 1
                return None
        else:
            # Truly cold start (home-seed disabled): the drone is the LARGEST
            # compact cluster above ground.
            candidates.sort(key=lambda c: c[2], reverse=True)

        self._last_sel_n = candidates[0][2]
        if self._dump is not None:
            self._last_sel_mask = labels == candidates[0][0]
            self._last_sel_pts = pts[self._last_sel_mask]
        return candidates[0][1]

    # ------------------------------------------------------------------
    # Coast (predict-only when no detection)
    # ------------------------------------------------------------------
    def _coast(self, header: Header):
        if not self._kf.initialized:
            return
        self._kf.predict(self._dt)
        self._coast_count += 1
        if self._coast_count <= self.coast_max:
            self._publish_pose(header)
        elif self._coast_count > self.reset_after:  # 기본값 50프레임 = 5초
            self.get_logger().warn(
                f'Tracking LOST {self._coast_count}프레임 → KF 리셋, 재획득 대기')
            self._kf.initialized = False
            self._coast_count = 0
            self._finalize_waypoint_episode()
        else:
            self.get_logger().warn(
                f'Tracking LOST: {self._coast_count} coast frames.',
                throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_pose(self, header: Header):
        pos = self._kf.position
        cov_pos = self._kf.cov_pos

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.w = 1.0  # yaw unknown from position-only observations

        # 6×6 flat covariance (pose: x,y,z,rx,ry,rz)
        cov = [0.0] * 36
        for i in range(3):
            cov[i * 6 + i] = float(cov_pos[i, i])
        msg.pose.covariance = cov

        self._pub_pose.publish(msg)

    # ------------------------------------------------------------------
    # Waypoint(마커) 호버 감지 + 위치 재추정
    # ------------------------------------------------------------------
    def _update_waypoint_tracking(self, centroid: np.ndarray):
        """가장 가까운 마커 반경 안에서 정지 상태가 유지되는 동안 raw
        centroid 표본을 모은다. KF 평활값이 아니라 raw 값을 쓰는 이유:
        정지 전환 구간에서 KF는 관성으로 인해 실제 위치보다 살짝 지연된
        값을 내므로, 여러 프레임의 raw 표본을 median으로 묶는 쪽이 더
        정확하다. 속도 판정만 KF의 평활 속도를 쓴다 (raw centroid 프레임간
        노이즈만으로는 속도 추정이 너무 불안정함).

        속도추정이 정지 중에도 프레임 단위로 미세하게 흔들려서 문턱값을
        순간적으로 넘나드는 경우가 실측에서 확인됨 — 그때마다 즉시
        끊어버리면 진짜 하나의 호버가 여러 조각으로 쪼개진다. 그래서
        이탈이 waypoint_grace_frames 이내로 짧으면 무시하고 같은 방문으로
        이어붙이고, 그보다 길게 지속돼야 실제로 떠난 것으로 본다."""
        speed = float(np.linalg.norm(self._kf.velocity[:2]))  # 수평 속도만
        slow = speed <= self.waypoint_speed_threshold

        if self._wp_active_id is not None:
            # 이미 특정 마커를 추적 중이면, "가장 가까운 마커"를 새로 찾지
            # 않고 지금 추적 중인 그 마커와의 거리만 본다 (경계에서 다른
            # 마커로 순간 전환되는 걸 방지).
            mx, my = self._marker_xy[self._wp_active_id]
            d = math.hypot(centroid[0] - mx, centroid[1] - my)
            near = d <= self.waypoint_gate_radius
            if near and slow:
                self._wp_samples.append(centroid.copy())
                self._wp_break_count = 0
                return
            self._wp_break_count += 1
            if self._wp_break_count <= self.waypoint_grace_frames:
                return  # 짧은 이탈 — 유예, 같은 방문 유지 (이 프레임 표본은 버림)
            self._finalize_waypoint_episode()
            # 이 프레임에 새 방문이 시작될 수도 있으니 아래로 계속 진행

        # 활성 방문이 없는 상태 — 새 방문 시작 여부 판정
        best_id, best_d = None, None
        for mid, (mx, my) in self._marker_xy.items():
            d = math.hypot(centroid[0] - mx, centroid[1] - my)
            if best_d is None or d < best_d:
                best_id, best_d = mid, d

        near = best_d is not None and best_d <= self.waypoint_gate_radius
        if near and slow:
            self._wp_active_id = best_id
            self._wp_samples = [centroid.copy()]
            self._wp_break_count = 0

    def _finalize_waypoint_episode(self):
        mid = self._wp_active_id
        samples = self._wp_samples
        self._wp_active_id = None
        self._wp_samples = []
        self._wp_break_count = 0
        if mid is None or len(samples) < self.waypoint_min_frames:
            return

        arr = np.array(samples)
        med = np.median(arr, axis=0)
        n = len(samples)
        duration = n / self.update_rate

        self._wp_visit_counts[mid] = self._wp_visit_counts.get(mid, 0) + 1
        visit = self._wp_visit_counts[mid]

        self.get_logger().info(
            f'[경로점] id{mid} 방문#{visit}: '
            f'({med[0]:.3f}, {med[1]:.3f}, {med[2]:.3f})  '
            f'n={n}샘플({duration:.1f}초)')

        self._wp_csv_w.writerow([
            mid, visit, f'{med[0]:.4f}', f'{med[1]:.4f}', f'{med[2]:.4f}',
            n, f'{duration:.2f}'])
        self._wp_csv_f.flush()

        self._publish_waypoint_marker(mid, visit, med)

    def _publish_waypoint_marker(self, marker_id: int, visit: int, pos: np.ndarray):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'waypoint_estimates'
        m.id = marker_id * 100 + visit
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 0.9
        self._wp_markers.markers.append(m)
        self._pub_waypoints.publish(self._wp_markers)

    def close(self):
        """CSV 마무리 저장. main()의 종료 처리에서 호출."""
        self._finalize_waypoint_episode()
        try:
            self._wp_csv_f.close()
        except Exception:
            pass
        self.get_logger().info(f'경로점 추정 결과 저장: {self._wp_csv_path}')
        self._save_cluster_dump()

    def _save_cluster_dump(self):
        """클러스터 점들을 npz 하나로 저장.

        프레임마다 점 개수가 달라 2D 배열로 못 쌓는다. 전부 이어붙인 뒤
        프레임 경계를 offsets로 따로 기록하는 방식(CSR과 같은 구조)을 쓴다."""
        if not self._dump:
            return
        times = np.array([d[0] for d in self._dump], dtype=np.float64)
        counts = np.array([len(d[1]) for d in self._dump], dtype=np.int32)
        pts = np.concatenate([d[1] for d in self._dump], axis=0)
        cents = np.stack([d[2] for d in self._dump], axis=0)
        srcs = np.concatenate([d[3] for d in self._dump], axis=0)
        offsets = np.concatenate([[0], np.cumsum(counts)]).astype(np.int64)
        path = os.path.expanduser(self.cluster_dump_path)
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        np.savez_compressed(path, times=times, points=pts, offsets=offsets,
                            centroids=cents, sources=srcs)
        n1 = int((srcs == 0).sum())
        n2 = int((srcs == 1).sum())
        self.get_logger().info(
            f'클러스터 덤프 저장: {path} '
            f'({len(times)}프레임, 점 {len(pts)}개 — '
            f'{self._sensors[0].name} {n1}개 / {self._sensors[1].name} {n2}개)')

    # ------------------------------------------------------------------
    # PointCloud2 utilities
    # ------------------------------------------------------------------
    def _pc2_to_numpy(self, msg: PointCloud2) -> np.ndarray | None:
        """Return Nx3 float32 in sensor frame, non-finite points stripped.

        sensor_msgs_py.read_points returns a *structured* numpy array
        (fields 'x','y','z'), which cannot be cast to float32 directly.
        We stack the named fields into an Nx3 array instead.

        Non-returning LiDAR rays come back as +/-inf (not NaN), so
        skip_nans does NOT remove them — we filter with np.isfinite.
        """
        try:
            arr = pc2_util.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if arr is None or arr.size == 0:
                return None
            pts = np.stack(
                [arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float32)
            pts = pts.reshape(-1, 3)
            # Drop inf (rays that hit nothing within range)
            finite = np.isfinite(pts).all(axis=1)
            pts = pts[finite]
            if pts.shape[0] == 0:
                return None
            return pts
        except Exception as e:
            self.get_logger().error(
                f'PC2 read error: {e}', throttle_duration_sec=5.0)
            return None

    def _numpy_to_pc2(self, pts: np.ndarray, ref_header: Header) -> PointCloud2:
        h = Header()
        h.stamp = ref_header.stamp
        h.frame_id = 'map'
        return pc2_util.create_cloud_xyz32(h, pts.tolist())


# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = None
    try:
        node = LidarDroneTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
