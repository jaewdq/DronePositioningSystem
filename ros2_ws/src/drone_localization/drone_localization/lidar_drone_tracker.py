#!/usr/bin/env python3
"""
Two-LiDAR drone 3D localization node (outside-in, ground-fixed sensors).

Two ground LiDARs (east + north, perpendicular views) each publish a
PointCloud2. Each callback transforms its cloud into the shared Gazebo
world frame and buffers it. A fixed-rate timer then fuses the buffers and
runs one pipeline + EKF step:

  /ouster1/points ─┐ (→ world frame, buffered)
                   ├─ concat ─→ ROI filter
  /ouster2/points ─┘             → voxel downsampling (open3d)
                                 → RANSAC ground removal (open3d)
                                 → statistical outlier removal (rain/scatter)
                                 → DBSCAN clustering (open3d, PCL-equivalent)
                                 → drone cluster selection
                                 → EKF predict+update (6-state const-velocity)
                                 → PoseWithCovarianceStamped /drone/estimated_pose

Fusing two perpendicular views triangulates the target in 3D and roughly
doubles the number of hits on the small (~50cm) drone.

All spatial operations are in the Gazebo world frame (ENU, Z-up).
Field spans X:[0,30] Y:[0,20]; field center = (15, 10).
Each LiDAR must be outside the 33x23m exclusion zone around field center.
Sensor poses MUST match the LIDARS list in generate_competition_map.py.

Dependencies (install once):
    pip3 install open3d scipy
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

try:
    from scipy.spatial.transform import Rotation as R
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    import open3d as o3d
    HAS_O3D = True
except ImportError:
    HAS_O3D = False

# Use sensor_msgs_py if available (ROS2 Humble ships it), fall back to sensor_msgs
try:
    from sensor_msgs_py import point_cloud2 as pc2_util
except ImportError:
    from sensor_msgs import point_cloud2 as pc2_util


# ---------------------------------------------------------------------------
# EKF: 6-state (x, y, z, vx, vy, vz), constant-velocity model
# ---------------------------------------------------------------------------
class EKF6D:
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
        K = self.P @ self.H.T @ np.linalg.inv(S)        # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

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
    # Field center in Gazebo world frame (field spans [0,30]x[0,20])
    _FIELD_CX = 15.0
    _FIELD_CY = 10.0
    # Half-extents of the 33x23m exclusion zone around the field center
    _EXCL_HX = 16.5
    _EXCL_HY = 11.5

    def __init__(self):
        super().__init__('lidar_drone_tracker')

        if not HAS_O3D:
            self.get_logger().fatal(
                'open3d not installed. Run: pip3 install open3d')
            raise SystemExit(1)
        if not HAS_SCIPY:
            self.get_logger().fatal(
                'scipy not installed. Run: pip3 install scipy')
            raise SystemExit(1)

        self._declare_parameters()
        self._read_parameters()
        self._build_sensors()          # builds + validates all sensor poses

        self._ekf = EKF6D(
            self.proc_noise_pos, self.proc_noise_vel, self.meas_noise_pos)
        # Seed the filter at the known home pad so tracking is locked from the
        # ground and the selection prefers the cluster nearest to the pad.
        if self.seed_at_home:
            self._ekf.init(self.home_xyz.copy())
        self._coast_count = 0

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
        self._subs = []
        for i, cfg in enumerate(self._sensors):
            self._subs.append(self.create_subscription(
                PointCloud2, cfg.topic,
                self._make_pc_callback(i), sensor_qos))

        self._pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/drone/estimated_pose', reliable_qos)
        self._pub_filtered = self.create_publisher(
            PointCloud2, '/lidar/filtered_points', sensor_qos)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/lidar/cluster_markers', reliable_qos)

        # Single processing clock at the LiDAR rate: fuse both buffers,
        # run the pipeline, and step the EKF at a fixed dt. Decoupling the
        # fusion from the two async callbacks keeps EKF timing consistent.
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
        self.declare_parameter('lidar1_topic', '/ouster1/points')
        self.declare_parameter('lidar1_x',     35.0)
        self.declare_parameter('lidar1_y',     10.0)
        self.declare_parameter('lidar1_z',      2.5)
        self.declare_parameter('lidar1_roll',   0.0)
        self.declare_parameter('lidar1_pitch',  0.0)
        self.declare_parameter('lidar1_yaw',    math.pi)

        # --- LiDAR 2 pose (perpendicular view for triangulation) ---
        # Default: north side, y=25 (outside 21.5m boundary), facing south.
        # Set enable_lidar2:=false to run with a single LiDAR.
        self.declare_parameter('enable_lidar2', True)
        self.declare_parameter('lidar2_topic', '/ouster2/points')
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

        # ROI in world frame — 2m margin around the 30×20 field
        self.declare_parameter('roi_x_min', -2.0)
        self.declare_parameter('roi_x_max', 32.0)
        self.declare_parameter('roi_y_min', -2.0)
        self.declare_parameter('roi_y_max', 22.0)
        self.declare_parameter('roi_z_min',  0.0)
        self.declare_parameter('roi_z_max',  6.0)

        # RANSAC ground removal
        # threshold=5cm: captures grid-line decals (effectively z≈0) while
        # keeping the drone (≥0.5m altitude) as an outlier to preserve.
        # Only removes the plane if it captures ≥20% of ROI points, to
        # avoid accidentally decimating a drone-body false-positive plane.
        self.declare_parameter('ransac_distance_threshold', 0.05)
        self.declare_parameter('ransac_max_iterations',     100)
        self.declare_parameter('ground_inlier_min_ratio',   0.20)

        # Statistical outlier removal (rain / scatter noise)
        self.declare_parameter('sor_mean_k',     20)
        self.declare_parameter('sor_std_ratio',   2.0)

        # DBSCAN clustering (PCL EuclideanClusterExtraction equivalent)
        # eps=25cm: connects points within the same drone body (50cm–1m)
        # min_pts=3: side-view LiDAR gives few hits on a small drone at distance
        self.declare_parameter('cluster_tolerance',  0.25)
        self.declare_parameter('cluster_min_points',  3)
        self.declare_parameter('cluster_max_points',  400)

        # Height filter (world Z). Lowered to 0.05 so the drone SITTING ON
        # the pad (centroid ~0.15m) is recognised from the start, not just
        # once airborne. Ground-fixed obstacles are instead rejected by the
        # home-seed + nearest-to-track selection (see below).
        self.declare_parameter('min_drone_height', 0.05)
        self.declare_parameter('max_drone_height', 5.5)

        # Home-seed: the drone always starts at a known pad. Seeding the EKF
        # there lets tracking lock on immediately (even on the ground) and
        # makes the nearest-to-track selection reject a distant ground box.
        self.declare_parameter('seed_at_home', True)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', 0.15)

        # EKF noise
        self.declare_parameter('proc_noise_pos',  0.05)
        self.declare_parameter('proc_noise_vel',  0.30)
        self.declare_parameter('meas_noise_pos',  0.15)

        # Gating: reject a candidate farther than this from the EKF prediction
        # (metres). Stops the airborne track from snapping onto a low ground
        # residual now that min_drone_height is small. Must exceed the distance
        # the drone can travel during a coast: 0.75 m/s × 1.5 s ≈ 1.1 m.
        self.declare_parameter('max_assoc_dist', 3.0)

        # Coasting: max frames to continue pure-predict when no detection
        self.declare_parameter('coast_max_frames', 5)

        self.declare_parameter('lidar_update_rate', 10.0)

    def _read_parameters(self):
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.enable_lidar2 = gp('enable_lidar2')
        self.buffer_stale_timeout = gp('buffer_stale_timeout')
        self.voxel_leaf_size = gp('voxel_leaf_size')
        self.roi_x_min = gp('roi_x_min')
        self.roi_x_max = gp('roi_x_max')
        self.roi_y_min = gp('roi_y_min')
        self.roi_y_max = gp('roi_y_max')
        self.roi_z_min = gp('roi_z_min')
        self.roi_z_max = gp('roi_z_max')
        self.ransac_dist  = gp('ransac_distance_threshold')
        self.ransac_iters = gp('ransac_max_iterations')
        self.ground_ratio = gp('ground_inlier_min_ratio')
        self.sor_k        = gp('sor_mean_k')
        self.sor_std      = gp('sor_std_ratio')
        self.eps          = gp('cluster_tolerance')
        self.min_pts      = gp('cluster_min_points')
        self.max_pts      = gp('cluster_max_points')
        self.min_h        = gp('min_drone_height')
        self.max_h        = gp('max_drone_height')
        self.seed_at_home = gp('seed_at_home')
        self.home_xyz     = np.array(
            [gp('home_x'), gp('home_y'), gp('home_z')])
        self.max_assoc_dist = gp('max_assoc_dist')
        self.proc_noise_pos = gp('proc_noise_pos')
        self.proc_noise_vel = gp('proc_noise_vel')
        self.meas_noise_pos = gp('meas_noise_pos')
        self.coast_max    = gp('coast_max_frames')
        self.update_rate  = gp('lidar_update_rate')
        self._dt = 1.0 / self.update_rate

    def _build_sensors(self):
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        specs = [(
            'lidar1', gp('lidar1_topic'), gp('lidar1_x'), gp('lidar1_y'),
            gp('lidar1_z'), gp('lidar1_roll'), gp('lidar1_pitch'),
            gp('lidar1_yaw'))]
        if self.enable_lidar2:
            specs.append((
                'lidar2', gp('lidar2_topic'), gp('lidar2_x'), gp('lidar2_y'),
                gp('lidar2_z'), gp('lidar2_roll'), gp('lidar2_pitch'),
                gp('lidar2_yaw')))

        self._sensors = []
        for name, topic, x, y, z, roll, pitch, yaw in specs:
            self._validate_pose(name, x, y)
            self._sensors.append(SensorCfg(name, topic, x, y, z, roll, pitch, yaw))

    def _validate_pose(self, name, x, y):
        dx = abs(x - self._FIELD_CX)
        dy = abs(y - self._FIELD_CY)
        if dx < self._EXCL_HX and dy < self._EXCL_HY:
            self.get_logger().fatal(
                f'{name} position ({x:.2f}, {y:.2f}) falls inside the '
                f'33x23m exclusion zone centred at '
                f'({self._FIELD_CX}, {self._FIELD_CY}). '
                f'Required: |x-{self._FIELD_CX}|>{self._EXCL_HX} OR '
                f'|y-{self._FIELD_CY}|>{self._EXCL_HY}. '
                'Valid examples: (35,10) east, (-2,10) west, (15,25) north, '
                '(15,-2) south.'
            )
            raise SystemExit(1)

    # ------------------------------------------------------------------
    # Per-sensor callback: store the latest cloud in WORLD frame
    # ------------------------------------------------------------------
    def _make_pc_callback(self, index: int):
        cfg = self._sensors[index]

        def _cb(msg: PointCloud2):
            pts = self._pc2_to_numpy(msg)
            if pts is None or len(pts) == 0:
                return
            world = cfg.to_world(pts)
            self._buffers[index] = (world, time.monotonic())

        return _cb

    # ------------------------------------------------------------------
    # Fusion + pipeline + EKF, driven by a fixed-rate timer
    # ------------------------------------------------------------------
    def _process(self):
        now = time.monotonic()
        clouds = []
        for i, buf in enumerate(self._buffers):
            if buf is None:
                continue
            pts, t = buf
            if now - t <= self.buffer_stale_timeout:
                clouds.append(pts)
                self._stat_sensor_pts[i] += len(pts)
                self._stat_sensor_hits[i] += 1

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        if not clouds:
            self._coast(header)
        else:
            pts_w = np.concatenate(clouds, axis=0)
            self._process_cloud(pts_w, header)

        self._maybe_log_stats(now)

    def _maybe_log_stats(self, now: float):
        if now - self._stat_t0 < self._stat_period:
            return
        f = self._stat_frames
        if f == 0:
            self.get_logger().warn(
                'No LiDAR input in the last '
                f'{self._stat_period:.0f}s — check /ouster1,2/points and Agent.')
        else:
            det = self._stat_detect
            rate = 100.0 * det / f
            avg_pts = (self._stat_cluster_pts / det) if det else 0.0
            avg_cand = self._stat_cand / f
            avg_merged = self._stat_merged_pts / f
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
        self._stat_t0 = now
        self._stat_frames = self._stat_detect = 0
        self._stat_cluster_pts = self._stat_cand = self._stat_merged_pts = 0
        n = len(self._sensors)
        self._stat_sensor_pts = [0] * n
        self._stat_sensor_hits = [0] * n

    def _process_cloud(self, pts_w: np.ndarray, header: Header):
        """Run the full pipeline on the fused world-frame cloud, step EKF."""
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
        if len(pts_w) < 10:
            self._coast(header)
            return

        # Open3D processing: voxel → ground removal → outlier removal
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_w.astype(np.float64))

        pcd = pcd.voxel_down_sample(self.voxel_leaf_size)
        pcd = self._remove_ground(pcd)

        # SOR targets rain/scatter noise. Only run it when the post-ground
        # cloud is dense enough to clearly BE clutter — in clear air only the
        # drone's few points remain, and SOR could delete that sparse cluster
        # (a 2-3 point drone at long range would be wrongly flagged outlier).
        if len(pcd.points) > max(self.sor_k * 5, 100):
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=self.sor_k, std_ratio=self.sor_std)

        # Publish filtered cloud for RViz2 debugging
        pts_f = np.asarray(pcd.points, dtype=np.float32)
        if len(pts_f) > 0:
            self._pub_filtered.publish(self._numpy_to_pc2(pts_f, header))

        # DBSCAN clustering
        if len(pcd.points) < self.min_pts:
            self._coast(header)
            return

        labels = np.array(
            pcd.cluster_dbscan(
                eps=self.eps,
                min_points=self.min_pts,
                print_progress=False,
            )
        )

        # Select the drone cluster
        centroid = self._select_drone_cluster(pts_f, labels, header)
        self._stat_cand += self._last_n_cand

        # EKF predict (fixed dt from the processing timer) → update
        self._ekf.predict(self._dt)

        if centroid is not None:
            self._stat_detect += 1
            self._stat_cluster_pts += self._last_sel_n
            if not self._ekf.initialized:
                self._ekf.init(centroid)
            else:
                self._ekf.update(centroid)
            self._coast_count = 0
        else:
            self._coast_count += 1
            if self._coast_count > self.coast_max:
                self.get_logger().warn(
                    f'Tracking LOST: {self._coast_count} consecutive '
                    'missed frames.', throttle_duration_sec=2.0)
                return

        if self._ekf.initialized:
            self._publish_pose(header)

    # ------------------------------------------------------------------
    # Ground removal
    # ------------------------------------------------------------------
    def _remove_ground(self, pcd: 'o3d.geometry.PointCloud') -> 'o3d.geometry.PointCloud':
        n_total = len(pcd.points)
        if n_total < 4:
            return pcd

        try:
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=self.ransac_dist,
                ransac_n=3,
                num_iterations=self.ransac_iters,
            )
        except Exception:
            return pcd

        # Guard 1: not enough inliers to confidently call it the ground plane
        if len(inliers) < n_total * self.ground_ratio:
            return pcd

        # Guard 2: plane normal must be roughly vertical (|nz| > cos 45°)
        # This prevents removing a large vertical structure (wall, obstacle)
        # that RANSAC might fit when the drone is absent and the scene is sparse.
        normal = np.asarray(plane_model[:3])
        if np.linalg.norm(normal) > 0:
            normal = normal / np.linalg.norm(normal)
        if abs(normal[2]) < 0.7:
            return pcd

        return pcd.select_by_index(inliers, invert=True)

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
            return None

        candidates = []
        marker_msgs = MarkerArray()

        for label in unique:
            mask = labels == label
            cluster = pts[mask]
            n = len(cluster)
            if n < self.min_pts or n > self.max_pts:
                continue

            centroid = cluster.mean(axis=0)

            # Height filter: excludes ground-fixed obstacles (box at floor level)
            if centroid[2] < self.min_h or centroid[2] > self.max_h:
                continue

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
            return None

        if self._ekf.initialized:
            # Prefer the cluster nearest to the current EKF estimate (which is
            # seeded at home, so this locks onto the pad drone from the start
            # and rejects a distant ground obstacle). Still valid while coasting
            # — the predicted position is a good prior.
            ekf_pos = self._ekf.position
            candidates.sort(key=lambda c: float(np.linalg.norm(c[1] - ekf_pos)))
            # Gate: if even the nearest candidate is too far, it isn't the drone.
            if float(np.linalg.norm(candidates[0][1] - ekf_pos)) > self.max_assoc_dist:
                return None
        else:
            # Truly cold start (home-seed disabled): the drone is the LARGEST
            # compact cluster above ground.
            candidates.sort(key=lambda c: c[2], reverse=True)

        self._last_sel_n = candidates[0][2]
        return candidates[0][1]

    # ------------------------------------------------------------------
    # Coast (predict-only when no detection)
    # ------------------------------------------------------------------
    def _coast(self, header: Header):
        if not self._ekf.initialized:
            return
        self._ekf.predict(self._dt)
        self._coast_count += 1
        if self._coast_count <= self.coast_max:
            self._publish_pose(header)
        else:
            self.get_logger().warn(
                f'Tracking LOST after {self._coast_count} coast frames.')

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_pose(self, header: Header):
        pos = self._ekf.position
        cov_pos = self._ekf.cov_pos

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
    try:
        node = LidarDroneTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        rclpy.try_shutdown()
