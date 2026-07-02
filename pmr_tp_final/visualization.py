"""RViz visualization helpers for the local CBF controller."""

from typing import Tuple

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from pmr_tp_final.robot_model import Robot


Color = Tuple[float, float, float, float]


class ControllerVisualizer:
    """Publish simple debug markers for CBF controller outputs."""

    def __init__(self, node: Node, robot_name: str, fixed_z: float) -> None:
        self._node = node
        self._robot_name = robot_name
        self._fixed_z = fixed_z
        self._frame_id = "world"
        self._trace_path = Path()
        self._trace_path.header.frame_id = self._frame_id

        prefix = f"/{robot_name}/viz"
        self._trace_pub = node.create_publisher(
            Path,
            f"{prefix}/trace_path",
            10,
        )
        self._accel_pub = node.create_publisher(
            MarkerArray,
            f"{prefix}/accelerations",
            10,
        )
        self._safety_pub = node.create_publisher(
            MarkerArray,
            f"{prefix}/safety_disk",
            10,
        )

    def publish_trace_path(self, ego: Robot) -> None:
        """Publish the measured path traced by the ego robot."""
        stamp = self._node.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._frame_id
        pose.pose.position = self._point(ego.position)
        pose.pose.orientation.w = 1.0

        self._trace_path.header.stamp = stamp
        self._trace_path.poses.append(pose)
        self._trace_pub.publish(self._trace_path)

    def publish_accelerations(
        self,
        ego: Robot,
        nominal_accel: np.ndarray,
        filtered_accel: np.ndarray,
        nominal_scale: float = 1.0,
        input_scale: float = 2.0,
    ) -> None:
        """Publish nominal and QP-filtered accelerations as arrows."""
        markers = MarkerArray()
        origin = ego.position
        markers.markers.append(
            self._arrow_marker(
                marker_id=0,
                start=origin,
                vector=nominal_scale * nominal_accel,
                color=(1.0, 0.6, 0.0, 1.0),
                namespace="nominal_accel",
            )
        )
        markers.markers.append(
            self._arrow_marker(
                marker_id=1,
                start=origin,
                vector=input_scale * filtered_accel,
                color=(0.0, 0.9, 0.2, 1.0),
                namespace="filtered_accel",
            )
        )
        self._accel_pub.publish(markers)

    def publish_safety_disk(
        self,
        ego: Robot,
        safety_margin: float,
    ) -> None:
        """Publish the inflated safety disk for the ego robot."""
        markers = MarkerArray()
        markers.markers.append(
            self._cylinder_marker(
                marker_id=0,
                center=ego.position,
                radius=ego.radius + safety_margin,
                color=(0.1, 0.6, 1.0, 0.22),
                namespace="safety_disk",
            )
        )
        self._safety_pub.publish(markers)

    def publish_all(
        self,
        ego: Robot,
        nominal_accel: np.ndarray,
        filtered_accel: np.ndarray,
        safety_margin: float,
    ) -> None:
        """Publish every visualization layer used by the controller."""
        self.publish_trace_path(ego)
        self.publish_accelerations(ego, nominal_accel, filtered_accel)
        self.publish_safety_disk(ego, safety_margin)

    def _arrow_marker(
        self,
        marker_id: int,
        start: np.ndarray,
        vector: np.ndarray,
        color: Color,
        namespace: str,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.ARROW, color)
        marker.scale.x = 0.025
        marker.scale.y = 0.055
        marker.scale.z = 0.08
        marker.points = [self._point(start), self._point(start + vector)]
        return marker

    def _cylinder_marker(
        self,
        marker_id: int,
        center: np.ndarray,
        radius: float,
        color: Color,
        namespace: str,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.CYLINDER, color)
        marker.pose.position = self._point(center)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = 0.02
        return marker

    def _base_marker(
        self,
        marker_id: int,
        namespace: str,
        marker_type: int,
        color: Color,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime.sec = 1
        return marker

    def _point(self, xy: np.ndarray) -> Point:
        point = Point()
        point.x = float(xy[0])
        point.y = float(xy[1])
        point.z = float(self._fixed_z)
        return point
