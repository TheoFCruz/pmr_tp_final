"""RViz visualization helpers for the local CBF controller."""

import math
from typing import Dict, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from pmr_tp_final.robot_model import Robot


Color = Tuple[float, float, float, float]


class ControllerVisualizer:
    """Publish debug markers for reference tracking and CBF geometry."""

    def __init__(self, node: Node, robot_name: str, fixed_z: float) -> None:
        self._node = node
        self._robot_name = robot_name
        self._fixed_z = fixed_z
        self._frame_id = "world"

        prefix = f"/{robot_name}/viz"
        self._reference_pose_pub = node.create_publisher(
            PoseStamped,
            f"{prefix}/reference_pose",
            10,
        )
        self._reference_path_pub = node.create_publisher(
            Path,
            f"{prefix}/reference_path",
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
        self._vo_pub = node.create_publisher(
            MarkerArray,
            f"{prefix}/vo_cones",
            10,
        )

    def publish_reference(
        self,
        position_ref: np.ndarray,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> None:
        """Publish the moving reference point and nominal straight path."""
        stamp = self._node.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._frame_id
        pose.pose.position = self._point(position_ref)
        pose.pose.orientation.w = 1.0
        self._reference_pose_pub.publish(pose)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self._frame_id
        for point_xy in (start, goal):
            path_pose = PoseStamped()
            path_pose.header.stamp = stamp
            path_pose.header.frame_id = self._frame_id
            path_pose.pose.position = self._point(point_xy)
            path_pose.pose.orientation.w = 1.0
            path.poses.append(path_pose)
        self._reference_path_pub.publish(path)

    def publish_accelerations(
        self,
        ego: Robot,
        nominal_accel: np.ndarray,
        filtered_accel: np.ndarray,
        scale: float = 1.2,
    ) -> None:
        """Publish nominal and QP-filtered accelerations as arrows."""
        markers = MarkerArray()
        origin = ego.position
        markers.markers.append(
            self._arrow_marker(
                marker_id=0,
                start=origin,
                vector=scale * nominal_accel,
                color=(1.0, 0.6, 0.0, 1.0),
                namespace="nominal_accel",
            )
        )
        markers.markers.append(
            self._arrow_marker(
                marker_id=1,
                start=origin,
                vector=scale * filtered_accel,
                color=(0.0, 0.9, 0.2, 1.0),
                namespace="filtered_accel",
            )
        )
        self._accel_pub.publish(markers)

    def publish_safety_disk(
        self,
        ego: Robot,
        neighbors: Dict[str, Robot],
        safety_margin: float,
    ) -> None:
        """Publish the inflated safety disk for the ego robot."""
        del neighbors

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

    def publish_vo_cones(self, ego: Robot, neighbors: Dict[str, Robot]) -> None:
        """Publish circular-agent VO tangent rays and relative velocity arrows."""
        markers = MarkerArray()
        marker_id = 0
        for neighbor in neighbors.values():
            cone = self._vo_tangent_vectors(ego, neighbor)
            if cone is None:
                continue

            l_plus, l_minus = cone
            markers.markers.append(
                self._line_marker(
                    marker_id=marker_id,
                    start=ego.position,
                    end=ego.position + l_plus,
                    color=(0.8, 0.0, 1.0, 0.8),
                    namespace="vo_cone",
                )
            )
            marker_id += 1
            markers.markers.append(
                self._line_marker(
                    marker_id=marker_id,
                    start=ego.position,
                    end=ego.position + l_minus,
                    color=(0.8, 0.0, 1.0, 0.8),
                    namespace="vo_cone",
                )
            )
            marker_id += 1

            relative_velocity = neighbor.velocity - ego.velocity
            markers.markers.append(
                self._arrow_marker(
                    marker_id=marker_id,
                    start=ego.position,
                    vector=relative_velocity,
                    color=(1.0, 1.0, 0.0, 0.9),
                    namespace="relative_velocity",
                )
            )
            marker_id += 1

        self._vo_pub.publish(markers)

    def publish_all(
        self,
        ego: Robot,
        neighbors: Dict[str, Robot],
        start: np.ndarray,
        goal: np.ndarray,
        position_ref: np.ndarray,
        nominal_accel: np.ndarray,
        filtered_accel: np.ndarray,
        safety_margin: float,
    ) -> None:
        """Publish every visualization layer used by the controller."""
        self.publish_reference(position_ref, start, goal)
        self.publish_accelerations(ego, nominal_accel, filtered_accel)
        self.publish_safety_disk(ego, neighbors, safety_margin)
        self.publish_vo_cones(ego, neighbors)

    def _vo_tangent_vectors(
        self,
        ego: Robot,
        neighbor: Robot,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        p_ij = neighbor.position - ego.position
        distance = float(np.linalg.norm(p_ij))
        combined_radius = ego.radius + neighbor.radius
        if distance <= combined_radius or distance == 0.0:
            return None

        e = p_ij / distance
        e_perp = np.array([-e[1], e[0]], dtype=float)
        tangent_length = math.sqrt(distance**2 - combined_radius**2)
        along = (tangent_length**2 / distance) * e
        across = (combined_radius * tangent_length / distance) * e_perp
        return along + across, along - across

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

    def _line_marker(
        self,
        marker_id: int,
        start: np.ndarray,
        end: np.ndarray,
        color: Color,
        namespace: str,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.LINE_STRIP, color)
        marker.scale.x = 0.018
        marker.points = [self._point(start), self._point(end)]
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
