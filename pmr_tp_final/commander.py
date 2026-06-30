"""Crazyflie command helpers.

This module contains the bridge between the acceleration computed by the
CBF/QP controller and the ``cmd_full_state`` interface used by Crazyswarm2.
"""

import math
from typing import Optional, Tuple

import numpy as np
from crazyflie_interfaces.msg import FullState
from rclpy.node import Node
from rclpy.publisher import Publisher


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return a planar quaternion as ``(x, y, z, w)``."""
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class AccelerationCommander:
    """Publish one drone's acceleration commands as ``FullState``.

    The CBF/VO controller should compute acceleration in the horizontal plane.
    This class converts acceleration commands into an internally integrated
    double-integrator reference trajectory:

    ``v_ref[k+1] = v_ref[k] + a_des * dt``
    ``p_ref[k+1] = p_ref[k] + v_ref[k] * dt + 0.5 * a_des * dt**2``

    The first reference position is initialized from the measured position.
    After that, measured state feedback should be handled by the outer
    controller that computes ``a_des``; the full-state message remains a
    smooth, self-consistent trajectory for the Crazyflie inner loop to track.
    """

    def __init__(
        self,
        node: Node,
        robot_name: str,
        dt: float,
        fixed_z: float = 1.0,
        yaw: float = 0.0,
        max_velocity: Optional[float] = None,
        max_acceleration: Optional[float] = None,
        topic_suffix: str = "cmd_full_state",
    ) -> None:
        self._node = node
        self._robot_name = robot_name
        self._dt = dt
        self._fixed_z = fixed_z
        self._yaw = yaw
        self._max_velocity = max_velocity
        self._max_acceleration = max_acceleration
        self._p_ref_xy: Optional[np.ndarray] = None
        self._v_ref_xy: Optional[np.ndarray] = None
        self._publisher: Publisher = node.create_publisher(
            FullState,
            f"/{robot_name}/{topic_suffix}",
            10,
        )

    @property
    def robot_name(self) -> str:
        """Name of the Crazyflie controlled by this commander."""
        return self._robot_name

    def send_accel(
        self,
        p_meas: np.ndarray,
        v_meas: np.ndarray,
        a_des: np.ndarray,
        yaw: Optional[float] = None,
    ) -> FullState:
        """Publish acceleration command for one robot and return the message.

        ``p_meas``, ``v_meas``, and ``a_des`` may be 2D or 3D arrays. The
        current controller is expected to operate in the horizontal plane, so
        the generated command keeps altitude fixed at ``fixed_z``.
        """
        del v_meas

        p_xy = np.asarray(p_meas, dtype=float)[:2]
        a_xy = np.asarray(a_des, dtype=float)[:2]

        if self._p_ref_xy is None or self._v_ref_xy is None:
            self._p_ref_xy = p_xy.copy()
            self._v_ref_xy = np.zeros(2)

        a_xy = self._clip_norm(a_xy, self._max_acceleration)
        previous_v_ref_xy = self._v_ref_xy
        v_ref_xy = self._clip_norm(
            previous_v_ref_xy + a_xy * self._dt,
            self._max_velocity,
        )

        # If velocity clipping changed the next velocity, publish the effective
        # acceleration so position, velocity, and acceleration stay consistent.
        if self._dt > 0.0:
            a_xy = (v_ref_xy - previous_v_ref_xy) / self._dt

        p_ref_xy = (
            self._p_ref_xy
            + previous_v_ref_xy * self._dt
            + 0.5 * a_xy * self._dt**2
        )

        self._p_ref_xy = p_ref_xy
        self._v_ref_xy = v_ref_xy

        msg = self._make_full_state(
            p_ref_xy,
            v_ref_xy,
            a_xy,
            self._yaw if yaw is None else yaw,
        )
        self._publisher.publish(msg)
        return msg

    def set_dt(self, dt: float) -> None:
        """Update the integration timestep."""
        self._dt = dt

    def _make_full_state(
        self,
        p_xy: np.ndarray,
        v_xy: np.ndarray,
        a_xy: np.ndarray,
        yaw: float,
    ) -> FullState:
        msg = FullState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = float(p_xy[0])
        msg.pose.position.y = float(p_xy[1])
        msg.pose.position.z = float(self._fixed_z)

        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        msg.twist.linear.x = float(v_xy[0])
        msg.twist.linear.y = float(v_xy[1])
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = 0.0

        msg.acc.x = float(a_xy[0])
        msg.acc.y = float(a_xy[1])
        msg.acc.z = 0.0

        return msg

    @staticmethod
    def _clip_norm(vector: np.ndarray, max_norm: Optional[float]) -> np.ndarray:
        if max_norm is None:
            return vector

        norm = np.linalg.norm(vector)
        if norm <= max_norm or norm == 0.0:
            return vector

        return vector * (max_norm / norm)
