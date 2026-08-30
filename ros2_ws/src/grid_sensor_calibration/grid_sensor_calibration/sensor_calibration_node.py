"""ROS 2 node estimating one sensor's ``grid -> sensorN`` transform.

The node contains no calibration mathematics: it only handles parameters,
data acquisition, logging and persistence, and delegates every computation to
:mod:`grid_sensor_calibration.calibration_math`. One node instance calibrates
exactly one sensor, so any number of sensors can be calibrated by launching
more instances -- nothing about "two sensors" is hard coded.

Two data sources are supported:

* **Method A (offline YAML)** -- ``input_file`` holds the known grid targets and
  the recorded sensor measurements; calibration runs once at start-up.
* **Method B (live capture)** -- the node subscribes to the sensor detection
  topic through a :mod:`~grid_sensor_calibration.sensor_adapter` and exposes
  ``std_srvs/Trigger`` services to capture one correspondence at a time and to
  run the calibration when enough points have been collected.
"""

from __future__ import annotations

from collections import deque
from typing import Any, Deque, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .calibration_io import (
    CalibrationInput,
    CalibrationIOError,
    NominalPose,
    load_calibration_input,
    transform_from_estimate,
    update_calibration_file,
)
from .calibration_math import (
    CalibrationError,
    DEFAULT_MIN_BASELINE,
    DEFAULT_MIN_POINTS,
    RECOMMENDED_MIN_POINTS,
    SensorPoseEstimate,
    estimate_sensor_pose,
    installation_error,
    yaw_to_quaternion,
)
from .report import format_calibration_report, format_residual_table
from .sensor_adapter import Measurement, create_adapter

__all__ = ["SensorCalibrationNode", "main"]

#: QoS used for the latched calibration result topic.
_LATCHED_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_SAMPLE_BUFFER_SIZE = 200
#: Delay before the start-up calibration, so publishers/services are up first.
_AUTO_CALIBRATION_DELAY_SEC = 0.1


class SensorCalibrationNode(Node):
    """Estimates and stores the pose of a single sensor in the grid frame."""

    def __init__(self, **kwargs) -> None:
        """Declare parameters, wire up interfaces and optionally auto-calibrate.

        Args:
            **kwargs: Forwarded to ``rclpy.node.Node`` (e.g.
                ``parameter_overrides`` in unit tests).
        """
        super().__init__("sensor_calibration_node", **kwargs)

        self._declare_parameters()
        self.sensor_name: str = self.get_parameter("sensor_name").value
        self.parent_frame: str = self.get_parameter("parent_frame").value
        self.child_frame: str = (
            self.get_parameter("child_frame").value or self.sensor_name
        )
        self.minimum_number_of_points: int = max(
            int(self.get_parameter("minimum_number_of_points").value), DEFAULT_MIN_POINTS
        )
        self.minimum_baseline: float = float(self.get_parameter("minimum_baseline").value)
        self.method: str = self.get_parameter("method").value
        self.calibration_file: str = self.get_parameter("calibration_file").value

        self.nominal_pose = NominalPose(
            x=float(self.get_parameter("nominal_x").value),
            y=float(self.get_parameter("nominal_y").value),
            yaw=float(self.get_parameter("nominal_yaw").value),
        )

        self._grid_points: np.ndarray = np.empty((0, 2))
        self._sensor_points: np.ndarray = np.empty((0, 2))
        self._captured: List[np.ndarray] = []
        self._samples: Deque[Measurement] = deque(maxlen=_SAMPLE_BUFFER_SIZE)
        self._last_estimate: Optional[SensorPoseEstimate] = None

        self._load_dataset()
        self._setup_interfaces()

        self.get_logger().info(
            f"sensor_calibration_node ready for '{self.sensor_name}' "
            f"({self.parent_frame} -> {self.child_frame}), "
            f"{self._grid_points.shape[0]} known grid target(s)"
        )

        if self.get_parameter("auto_calibrate").value and self._sensor_points.size:
            # Run shortly after start-up, once publishers and services are up.
            self._auto_timer = self.create_timer(
                _AUTO_CALIBRATION_DELAY_SEC, self._run_auto_calibration
            )

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        """Declare every ROS parameter this node understands."""
        self.declare_parameter("sensor_name", "sensor1")
        self.declare_parameter("parent_frame", "grid")
        self.declare_parameter("child_frame", "")

        self.declare_parameter("nominal_x", 0.0)
        self.declare_parameter("nominal_y", 0.0)
        self.declare_parameter("nominal_yaw", 0.0)

        self.declare_parameter("minimum_number_of_points", DEFAULT_MIN_POINTS)
        self.declare_parameter("minimum_baseline", DEFAULT_MIN_BASELINE)
        self.declare_parameter("method", "closed_form")

        self.declare_parameter("input_file", "")
        self.declare_parameter("calibration_file", "calibration_result.yaml")
        self.declare_parameter("auto_calibrate", True)
        self.declare_parameter("save_result", True)
        self.declare_parameter("shutdown_after_calibration", False)
        self.declare_parameter("print_residual_table", True)

        # Inline dataset (flattened [x1, y1, x2, y2, ...]); overrides input_file.
        self.declare_parameter("grid_points", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("sensor_points", Parameter.Type.DOUBLE_ARRAY)

        # Live capture (method B).
        self.declare_parameter("input_topic", "")
        self.declare_parameter("input_message_type", "point_stamped")
        self.declare_parameter("capture_sample_count", 10)

    def _setup_interfaces(self) -> None:
        """Create the subscription, services and result publisher."""
        self.result_publisher = self.create_publisher(
            TransformStamped, "~/calibrated_transform", _LATCHED_QOS
        )

        adapter_name = self.get_parameter("input_message_type").value
        try:
            self._adapter = create_adapter(adapter_name, default_frame_id=self.child_frame)
        except ValueError as exc:
            self.get_logger().error(f"{exc}; falling back to 'point_stamped'")
            self._adapter = create_adapter("point_stamped", default_frame_id=self.child_frame)

        topic = self.get_parameter("input_topic").value or f"/{self.sensor_name}/target_point"
        self.create_subscription(self._adapter.message_type, topic, self._on_measurement, 10)
        self.get_logger().info(f"listening for calibration measurements on '{topic}'")

        self.create_service(Trigger, "~/capture_point", self._srv_capture_point)
        self.create_service(Trigger, "~/run_calibration", self._srv_run_calibration)
        self.create_service(Trigger, "~/reset", self._srv_reset)

    # ------------------------------------------------------------------
    # Dataset handling
    # ------------------------------------------------------------------

    def _flat_parameter_points(self, name: str) -> Optional[np.ndarray]:
        """Read a flattened ``[x1, y1, ...]`` parameter into an ``(N, 2)`` array.

        Args:
            name: Parameter name (``grid_points`` or ``sensor_points``).

        Returns:
            The ``(N, 2)`` array, or ``None`` when the parameter is unset,
            empty or has an odd number of values.
        """
        try:
            raw = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            return None
        if not raw:
            return None
        values = [float(value) for value in raw]
        if len(values) % 2 != 0:
            self.get_logger().error(
                f"parameter '{name}' must hold an even number of values "
                f"([x1, y1, x2, y2, ...]), got {len(values)}"
            )
            return None
        return np.asarray(values, dtype=float).reshape(-1, 2)

    def _load_dataset(self) -> None:
        """Populate the grid/sensor point arrays from parameters or YAML."""
        grid_from_param = self._flat_parameter_points("grid_points")
        sensor_from_param = self._flat_parameter_points("sensor_points")

        input_file = self.get_parameter("input_file").value
        dataset: Optional[CalibrationInput] = None
        if input_file:
            try:
                dataset = load_calibration_input(input_file, self.sensor_name)
            except CalibrationIOError as exc:
                self.get_logger().error(f"could not load input_file: {exc}")

        if dataset is not None:
            self._grid_points = dataset.grid_points
            self._sensor_points = dataset.sensor_points
            if not self.get_parameter("nominal_x").value and not self.get_parameter(
                "nominal_y"
            ).value:
                # Parameters left at their defaults: take the nominal pose from
                # the dataset instead.
                self.nominal_pose = dataset.nominal_pose
            self.get_logger().info(
                f"loaded {dataset.number_of_points} correspondence(s) from "
                f"'{dataset.source_path}'"
            )

        if grid_from_param is not None:
            self._grid_points = grid_from_param
        if sensor_from_param is not None:
            self._sensor_points = sensor_from_param

        if (
            self._grid_points.size
            and self._sensor_points.size
            and self._grid_points.shape != self._sensor_points.shape
        ):
            self.get_logger().error(
                f"grid_points {self._grid_points.shape} and sensor_points "
                f"{self._sensor_points.shape} disagree; live capture will be used instead"
            )
            self._sensor_points = np.empty((0, 2))

    # ------------------------------------------------------------------
    # Live capture (method B)
    # ------------------------------------------------------------------

    def _on_measurement(self, msg: Any) -> None:
        """Buffer an incoming detection for the next ``capture_point`` call.

        Args:
            msg: Driver message handled by the configured adapter.
        """
        measurement = self._adapter.extract(msg)
        if measurement is None:
            self.get_logger().warn("discarded a measurement with non-finite coordinates")
            return
        self._samples.append(measurement)

    def _srv_capture_point(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Capture the averaged buffered measurement as the next correspondence.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response; ``success`` is ``False`` when no samples
            arrived or when all known grid targets are already captured.
        """
        del request
        index = len(self._captured)
        if index >= self._grid_points.shape[0]:
            response.success = False
            response.message = (
                f"already captured {index} point(s); the configured grid_points list "
                f"holds only {self._grid_points.shape[0]} target(s)"
            )
            return response

        count = max(int(self.get_parameter("capture_sample_count").value), 1)
        samples = list(self._samples)[-count:]
        if not samples:
            response.success = False
            response.message = "no measurement received yet on the input topic"
            return response

        averaged = np.mean(np.vstack([sample.xy for sample in samples]), axis=0)
        spread = float(np.max(np.std(np.vstack([s.xy for s in samples]), axis=0))) if len(samples) > 1 else 0.0
        self._captured.append(averaged)
        self._samples.clear()

        target = self._grid_points[index]
        response.success = True
        response.message = (
            f"captured point {index + 1}/{self._grid_points.shape[0]}: "
            f"sensor ({averaged[0]:.3f}, {averaged[1]:.3f}) [std {spread:.4f} m over "
            f"{len(samples)} sample(s)] <-> grid ({target[0]:.3f}, {target[1]:.3f})"
        )
        self.get_logger().info(response.message)
        return response

    def _srv_reset(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Drop all captured correspondences and buffered samples.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response.
        """
        del request
        cleared = len(self._captured)
        self._captured.clear()
        self._samples.clear()
        response.success = True
        response.message = f"cleared {cleared} captured point(s)"
        self.get_logger().info(response.message)
        return response

    def _srv_run_calibration(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Run the calibration on the currently available correspondences.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response; ``message`` holds the one-line summary or
            the error reason.
        """
        del request
        try:
            estimate = self.calibrate()
        except (CalibrationError, CalibrationIOError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"calibration failed: {exc}")
            return response
        response.success = True
        response.message = (
            f"{self.sensor_name}: x={estimate.tx:.3f} m, y={estimate.ty:.3f} m, "
            f"yaw={estimate.yaw_deg:.3f} deg, rms={estimate.rms_error:.4f} m"
        )
        return response

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def _current_correspondences(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return the correspondence set to calibrate with.

        Live captured points take precedence over the configured dataset.

        Returns:
            Tuple ``(sensor_points, grid_points)`` as ``(N, 2)`` arrays.

        Raises:
            CalibrationError: If no usable correspondence set exists.
        """
        if self._captured:
            count = len(self._captured)
            sensor_points = np.vstack(self._captured)
            grid_points = self._grid_points[:count]
            return sensor_points, grid_points
        if self._sensor_points.size and self._grid_points.size:
            return self._sensor_points, self._grid_points
        raise CalibrationError(
            "no calibration data available: provide input_file / grid_points+sensor_points "
            "parameters, or capture points with the '~/capture_point' service"
        )

    def calibrate(self) -> SensorPoseEstimate:
        """Estimate the sensor pose, log the report and persist the result.

        Returns:
            The :class:`SensorPoseEstimate` for this sensor.

        Raises:
            CalibrationError: If the data are missing, too few or degenerate.
            CalibrationIOError: If the result file cannot be written.
        """
        sensor_points, grid_points = self._current_correspondences()

        if sensor_points.shape[0] < RECOMMENDED_MIN_POINTS:
            self.get_logger().warn(
                f"calibrating with only {sensor_points.shape[0]} point(s); "
                f"{RECOMMENDED_MIN_POINTS} or more well-spread targets are recommended"
            )

        estimate = estimate_sensor_pose(
            sensor_points,
            grid_points,
            minimum_number_of_points=self.minimum_number_of_points,
            minimum_baseline=self.minimum_baseline,
            method=self.method,
        )
        self._last_estimate = estimate

        error = installation_error(
            estimate.tx,
            estimate.ty,
            estimate.yaw,
            self.nominal_pose.x,
            self.nominal_pose.y,
            self.nominal_pose.yaw,
        )

        saved_path: Optional[str] = None
        if self.get_parameter("save_result").value:
            transform = transform_from_estimate(
                self.sensor_name,
                estimate,
                parent_frame=self.parent_frame,
                child_frame=self.child_frame,
                nominal=self.nominal_pose,
                error=error,
            )
            saved_path = update_calibration_file(
                self.calibration_file, self.sensor_name, transform
            )

        report = format_calibration_report(
            self.sensor_name,
            estimate,
            self.nominal_pose,
            error,
            parent_frame=self.parent_frame,
            child_frame=self.child_frame,
            output_path=saved_path,
        )
        if self.get_parameter("print_residual_table").value:
            report += "\n" + format_residual_table(estimate)
        self.get_logger().info(report)

        self.result_publisher.publish(self._to_transform_msg(estimate))
        return estimate

    def _to_transform_msg(self, estimate: SensorPoseEstimate) -> TransformStamped:
        """Convert an estimate into a ``grid -> sensor`` ``TransformStamped``.

        Args:
            estimate: The estimated planar pose.

        Returns:
            The stamped transform (z = 0, roll = pitch = 0).
        """
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame
        msg.transform.translation.x = float(estimate.tx)
        msg.transform.translation.y = float(estimate.ty)
        msg.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(estimate.yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def _run_auto_calibration(self) -> None:
        """One-shot timer callback running the start-up calibration."""
        self._auto_timer.cancel()
        try:
            self.calibrate()
        except (CalibrationError, CalibrationIOError) as exc:
            self.get_logger().error(f"automatic calibration failed: {exc}")
            return
        if self.get_parameter("shutdown_after_calibration").value:
            self.get_logger().info("shutdown_after_calibration is set; exiting")
            raise SystemExit(0)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for ``ros2 run grid_sensor_calibration sensor_calibration_node``.

    Args:
        args: Optional command line arguments forwarded to ``rclpy``.
    """
    rclpy.init(args=args)
    node: Optional[SensorCalibrationNode] = None
    try:
        node = SensorCalibrationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
