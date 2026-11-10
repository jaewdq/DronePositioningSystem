#!/usr/bin/env python3
"""
Flight recorder: logs the actual flown path and the LiDAR-estimated path
to CSV during a flight, so you can plot them afterwards with plot_flight.

Subscribes:
  /fmu/out/vehicle_local_position_v1  (PX4 ground-truth, NED)  → actual path
  /drone/estimated_pose               (LiDAR localization, world ENU) → estimate

Both are written in the Gazebo world frame (ENU) so they overlay directly
on the planned trajectory. PX4 local NED → world ENU:
  world_x = ned_y (east),  world_y = ned_x (north),  world_z = -ned_z (up)
(inverse of competition_mission's world_to_ned).

Rows are flushed as they arrive, so Ctrl+C (or a crash) still leaves a
complete CSV.

시작할 때 이전 기록이 남아 있으면 runNNN/ 으로 **자동 보관**한 뒤 새로
쓰기 시작한다 (auto_archive:=false 로 끌 수 있음). 이전에는 그냥
덮어썼기 때문에, 비행 후 archive_run을 깜빡하고 recorder를 다시 켜면
직전 비행 데이터가 사라졌다.

runNNN 번호는 **비행 하나에 하나만** 나간다. 추정 표본이 min_est_samples에
못 미치는 기록(착륙 후 Ctrl+C까지의 몇 초, 트래커 없이 돌린 구간)은 번호를
받지 못하고 _discarded/ 에 한 벌만 남는다.

Usage:
  ros2 run drone_localization flight_recorder
  # ... fly the mission (competition_mission) ...
  # Ctrl+C when done, then:
  ros2 run drone_localization plot_flight
"""

import csv
import os
import re
import shutil
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
)
from geometry_msgs.msg import PoseWithCovarianceStamped

try:
    from px4_msgs.msg import VehicleLocalPosition
    HAS_PX4 = True
except ImportError:
    HAS_PX4 = False


class FlightRecorder(Node):
    def __init__(self):
        super().__init__('flight_recorder')

        default_dir = os.path.expanduser('~/drone_project/flight_logs')
        self.declare_parameter('output_dir', default_dir)
        self.declare_parameter(
            'actual_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('estimate_topic', '/drone/estimated_pose')
        # 시작할 때 기존 로그를 runNNN/ 으로 자동 보관할지 여부.
        self.declare_parameter('auto_archive', True)
        # 착륙을 감지하면 그 비행을 보관하고 시각화를 자동으로 띄운다.
        self.declare_parameter('auto_plot', True)
        self.declare_parameter('plot_script', 'plot_accuracy.py')
        self.declare_parameter('takeoff_alt', 0.5)       # 이 위로 올라가면 이륙
        self.declare_parameter('land_settle_sec', 5.0)   # 착륙 후 대기
        # 아래 두 조건을 못 넘기면 비행으로 치지 않는다(착륙 직후 고도
        # 요동으로 한 비행이 조각나는 것을 막는 안전장치).
        self.declare_parameter('min_flight_sec', 20.0)
        self.declare_parameter('min_est_samples', 100)
        self.out_dir = self.get_parameter('output_dir').value
        os.makedirs(self.out_dir, exist_ok=True)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.auto_plot = gp('auto_plot')
        self.plot_script = gp('plot_script')
        self.takeoff_alt = gp('takeoff_alt')
        self.land_settle_sec = gp('land_settle_sec')
        self.min_flight_sec = gp('min_flight_sec')
        self.min_est_samples = gp('min_est_samples')

        self._actual_path = os.path.join(self.out_dir, 'actual_path.csv')
        self._est_path = os.path.join(self.out_dir, 'estimated_path.csv')

        # 비행 종료 감지 상태
        self._was_airborne = False
        self._landed_since = None
        self._airborne_since = None

        # 아래에서 'w'로 여는 순간 이전 비행 기록이 사라진다. 매번 수동으로
        # archive_run을 돌리는 걸 잊으면 그대로 유실되므로, 지우기 직전에
        # 자동으로 보관한다.
        if gp('auto_archive'):
            self._auto_archive()

        self._open_files()

        self._n_actual = 0
        self._n_est = 0
        # Single shared wall clock for BOTH streams. Using each message's own
        # embedded timestamp is wrong here: PX4 uses boot-microseconds and the
        # estimate uses ROS time — different clocks. Stamping both at reception
        # with one monotonic clock keeps them directly comparable (the <0.1s
        # LiDAR-processing latency is negligible for this analysis).
        self._t0 = time.monotonic()

        # PX4 publishes best-effort; estimate is reliable
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        est_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        if HAS_PX4:
            self.create_subscription(
                VehicleLocalPosition,
                self.get_parameter('actual_topic').value,
                self._actual_cb, px4_qos)
        else:
            self.get_logger().warn(
                'px4_msgs not found — actual (PX4) path will NOT be recorded. '
                'Only /drone/estimated_pose is logged.')

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('estimate_topic').value,
            self._est_cb, est_qos)

        self.get_logger().info(
            f'Recording flight to:\n  {self._actual_path}\n  {self._est_path}\n'
            'Fly the mission, then Ctrl+C and run: ros2 run drone_localization plot_flight')

    # ------------------------------------------------------------------
    # 시작 시 이전 로그 자동 보관
    # ------------------------------------------------------------------
    def _open_files(self):
        """CSV 두 개를 새로 열고 헤더를 쓴다. 비행이 끝날 때마다 재호출된다."""
        self._actual_f = open(self._actual_path, 'w', newline='')
        self._actual_w = csv.writer(self._actual_f)
        self._actual_w.writerow(['t_sec', 'world_x', 'world_y', 'world_z'])

        self._est_f = open(self._est_path, 'w', newline='')
        self._est_w = csv.writer(self._est_f)
        self._est_w.writerow(['t_sec', 'world_x', 'world_y', 'world_z'])

    def _n_rows(self, path):
        """헤더를 뺀 데이터 줄 수. 파일이 없으면 0."""
        if not os.path.isfile(path):
            return 0
        try:
            with open(path) as f:
                return max(sum(1 for _ in f) - 1, 0)
        except OSError:
            return 0

    def _next_run_dir(self):
        """archive_run과 같은 규칙으로 번호를 매겨 두 경로가 섞이지 않게 한다."""
        used = []
        for name in os.listdir(self.out_dir):
            m = re.fullmatch(r'run(\d+)', name)
            if m and os.path.isdir(os.path.join(self.out_dir, name)):
                used.append(int(m.group(1)))
        return os.path.join(self.out_dir,
                            f'run{max(used) + 1 if used else 1:03d}')

    def _auto_archive(self):
        n_act = self._n_rows(self._actual_path)
        n_est = self._n_rows(self._est_path)
        if n_act == 0 and n_est == 0:
            return None  # 보관할 게 없음 (첫 실행이거나 이미 옮겨감)

        # 비행 하나에 폴더 하나. 이 검사가 없던 시절에는 한 줄만 있어도
        # run 번호를 줬는데, 착륙을 감지해 보관한 뒤에도 레코더는 계속
        # 기록하므로 Ctrl+C까지의 몇 초가 곧바로 다음 run이 됐다(run055,
        # run057이 그렇게 생긴 3초짜리다). 트래커를 안 띄우고 돌린 구간도
        # 마찬가지로 추정이 빈 채 번호만 차지한다(run053). 그러면 나중에
        # 어느 게 진짜 비행인지 폴더 이름만으로는 알 수 없다.
        # _check_flight_end와 같은 기준으로 거른다.
        if n_est < self.min_est_samples:
            return self._discard(n_act, n_est)

        dest = self._next_run_dir()
        os.makedirs(dest, exist_ok=True)
        for src in (self._actual_path, self._est_path):
            if os.path.isfile(src):
                shutil.move(src, os.path.join(dest, os.path.basename(src)))

        self.get_logger().info(
            f'이전 기록을 자동 보관했습니다 → {dest} '
            f'(실측 {n_act}, 추정 {n_est}표본)')
        return dest

    def _discard(self, n_act, n_est):
        """비행으로 볼 수 없는 자투리 — run 번호는 주지 않는다.

        그렇다고 바로 지우지도 않는다. 추정이 비는 흔한 이유가 '트래커를
        안 띄웠다'인데 그 경우 실측 쪽은 멀쩡하고, 길게 돌린 기록이면
        아깝다. 한 벌만 _discarded/에 남기고 다음 자투리가 나오면 덮어쓴다.
        run(\\d+) 패턴이 아니라서 run 번호 매기기에도 끼어들지 않는다."""
        dest = os.path.join(self.out_dir, '_discarded')
        shutil.rmtree(dest, ignore_errors=True)
        os.makedirs(dest, exist_ok=True)
        for src in (self._actual_path, self._est_path):
            if os.path.isfile(src):
                shutil.move(src, os.path.join(dest, os.path.basename(src)))

        self.get_logger().info(
            f'비행으로 보기 어려워 run 번호를 주지 않았습니다 '
            f'(실측 {n_act}, 추정 {n_est}표본 < {self.min_est_samples}) → {dest}')
        return None      # None이므로 시각화도 안 뜬다

    # ------------------------------------------------------------------
    # 비행 종료 감지 → 보관 + 시각화 자동 실행
    # ------------------------------------------------------------------
    def _check_flight_end(self, alt: float, t: float):
        """이륙한 적이 있고, 착륙 후 조용한 시간이 지나면 한 비행으로 본다.

        미션 스크립트가 끝나는 시점을 구독하는 방법도 있지만, 수기 비행이나
        중간에 끊은 비행에는 안 통한다. 고도만 보면 어떻게 날렸든 동작한다.

        단순히 "고도가 문턱을 넘었다/내려왔다"만 보면 착륙 직후의 고도
        요동에 다시 이륙으로 반응해 한 비행이 여러 조각으로 잘린다(실제로
        2초짜리 run이 만들어졌다). 그래서 아래 두 조건을 함께 요구한다.
          - 충분히 오래 떠 있었을 것 (min_flight_sec)
          - 측위 표본이 충분히 쌓였을 것 (min_est_samples)
        둘 중 하나라도 모자라면 비행으로 치지 않고 계속 같은 파일에 쓴다."""
        if alt > self.takeoff_alt:
            if not self._was_airborne:
                self._airborne_since = t
            self._was_airborne = True
            self._landed_since = None
            return
        if not self._was_airborne:
            return                      # 아직 한 번도 안 떴음
        if self._landed_since is None:
            self._landed_since = t
            return
        if t - self._landed_since < self.land_settle_sec:
            return                      # 착륙 직후 — 조금 더 지켜본다

        flew = (self._landed_since - (self._airborne_since or 0.0))
        if flew < self.min_flight_sec or self._n_est < self.min_est_samples:
            # 짧게 튄 것이거나 측위가 거의 없었던 구간 — 비행으로 안 친다.
            self.get_logger().info(
                f'비행으로 보지 않음 (체공 {flew:.1f}초, 추정 {self._n_est}표본) '
                '— 계속 기록합니다.')
            self._was_airborne = False
            self._landed_since = None
            self._airborne_since = None
            return

        self.get_logger().info(
            f'비행 종료 감지 (체공 {flew:.0f}초, 추정 {self._n_est}표본) '
            '→ 보관 및 시각화')
        self._rotate_and_plot()

    def _rotate_and_plot(self):
        """현재 기록을 닫아 보관하고, 그 폴더로 시각화를 띄운 뒤 새로 시작한다."""
        try:
            self._actual_f.close()
            self._est_f.close()
        except Exception:
            pass

        dest = self._auto_archive()
        self._open_files()              # 다음 비행을 위해 새 파일로 재시작
        self._was_airborne = False
        self._landed_since = None
        self._airborne_since = None
        self._n_actual = self._n_est = 0
        self._t0 = time.monotonic()

        if dest and self.auto_plot:
            self._launch_plot(dest)

    def _launch_plot(self, run_dir):
        """시각화는 별도 프로세스로 띄운다. 노드 안에서 창을 열면 spin이
        막혀 다음 비행을 기록하지 못한다."""
        here = os.path.dirname(os.path.abspath(__file__))
        script = os.path.join(here, self.plot_script)
        if not os.path.isfile(script):
            self.get_logger().warn(f'시각화 스크립트 없음: {script}')
            return
        try:
            subprocess.Popen(
                [sys.executable, script, '--output_dir', run_dir],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.get_logger().info(
                f'시각화 실행: {self.plot_script} ({os.path.basename(run_dir)})')
        except Exception as e:
            self.get_logger().warn(f'시각화 실행 실패: {e}')

    def _actual_cb(self, msg: 'VehicleLocalPosition'):
        if not (msg.xy_valid and msg.z_valid):
            return
        t = time.monotonic() - self._t0        # shared clock
        # NED → world ENU
        world_x = msg.y
        world_y = msg.x
        world_z = -msg.z
        self._actual_w.writerow(
            [f'{t:.4f}', f'{world_x:.4f}', f'{world_y:.4f}', f'{world_z:.4f}'])
        self._actual_f.flush()
        self._n_actual += 1
        self._check_flight_end(world_z, t)

    def _est_cb(self, msg: PoseWithCovarianceStamped):
        t = time.monotonic() - self._t0        # same shared clock
        p = msg.pose.pose.position
        self._est_w.writerow(
            [f'{t:.4f}', f'{p.x:.4f}', f'{p.y:.4f}', f'{p.z:.4f}'])
        self._est_f.flush()
        self._n_est += 1

    def close(self):
        """Ctrl+C 종료. 착륙 감지 전에 끄는 경우(중간에 중단, 착륙 대기시간
        전에 종료)도 흔하므로, 여기서도 보관과 시각화를 한 번 해준다."""
        try:
            self._actual_f.close()
            self._est_f.close()
        except Exception:
            pass
        self.get_logger().info(
            f'Saved {self._n_actual} actual + {self._n_est} estimated samples.')

        dest = self._auto_archive()
        if dest and self.auto_plot:
            self._launch_plot(dest)


def main():
    rclpy.init()
    node = FlightRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
