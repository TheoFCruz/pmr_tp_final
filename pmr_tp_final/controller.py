"""Pseudo-decentralized ROS 2 node skeleton for Crazyflie CBF/QP control."""

from typing import Dict, List, Optional, Sequence

import numpy as np
import rclpy
from crazyflie_interfaces.msg import LogDataGeneric
from rclpy.node import Node

from pmr_tp_final.commander import AccelerationCommander
from pmr_tp_final.constraints import (
    BaseConstraint,
    SafetyCbfConstraint,
    VelocitySafetyCbfConstraint,
    VelocityObstacleCbfConstraint,
)
from pmr_tp_final.optimizer import AccelOptimizer, SolveStatus
from pmr_tp_final.robot_model import Robot


class CbfAgentController(Node):
    """Controller for one Crazyflie using neighbor states from ROS topics."""

    def __init__(
        self,
        robot_id: int = 0,
        robot_prefix: str = "cf_",
        total_robots: int = 4,
    ) -> None:
        super().__init__("cbf_agent_controller")

        self.declare_parameter("robot_id", robot_id)
        self.declare_parameter("robot_prefix", robot_prefix)
        self.declare_parameter("total_robots", total_robots)
        self.declare_parameter("control_period", 0.1)
        self.declare_parameter("fixed_z", 1.0)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("max_acceleration", 1.5)
        self.declare_parameter("robot_radius", 0.1)
        self.declare_parameter("safety_margin", 0.1)
        self.declare_parameter("safety_alpha", 1.0)
        self.declare_parameter("vo_alpha", 0.5)
        self.declare_parameter("vo_slack_weight_gain", 5.0)
        self.declare_parameter("scenario_radius", 1.0)

        self._robot_id = self.get_parameter("robot_id").value
        self._robot_prefix = self.get_parameter("robot_prefix").value
        self._total_robots = self.get_parameter("total_robots").value
        self._control_period = self.get_parameter("control_period").value
        self._robot_name = self._make_robot_name(self._robot_id)
        self._max_acceleration = self.get_parameter("max_acceleration").value
        self._robot_radius = self.get_parameter("robot_radius").value
        self._scenario_radius = float(self.get_parameter("scenario_radius").value)
        self._neighbor_names = self._make_neighbor_names()

        self._constraints: List[BaseConstraint] = [
            SafetyCbfConstraint(
                safety_margin=float(self.get_parameter("safety_margin").value),
                max_acceleration=float(self._max_acceleration),
                alpha=float(self.get_parameter("safety_alpha").value),
            ),
            VelocityObstacleCbfConstraint(
                alpha=float(self.get_parameter("vo_alpha").value),
                slack_weight_gain=float(
                    self.get_parameter("vo_slack_weight_gain").value
                ),
            ),
        ]

        self._optimizer = AccelOptimizer()
        self._optimizer.initialize_model(float(self._max_acceleration))

        self._states: Dict[str, Robot] = {}
        self._state_subscriptions = [
            self.create_subscription(
                LogDataGeneric,
                f"/{name}/state",
                lambda msg, agent_name=name: self._state_callback(agent_name, msg),
                10,
            )
            for name in self._all_agent_names()
        ]

        self._commander = AccelerationCommander(
            node=self,
            robot_name=self._robot_name,
            dt=self._control_period,
            fixed_z=self.get_parameter("fixed_z").value,
            max_velocity=self.get_parameter("max_velocity").value,
            max_acceleration=self.get_parameter("max_acceleration").value,
        )

        self._timer = self.create_timer(self._control_period, self._control_step)

        self.get_logger().info(
            f"CBF agent controller started for {self._robot_name}. "
            f"Neighbors: {self._neighbor_names}"
        )

    def _all_agent_names(self) -> List[str]:
        """Return ego plus neighbors without duplicates."""
        names = [self._robot_name]
        for name in self._neighbor_names:
            if name != self._robot_name and name not in names:
                names.append(name)
        return names

    def _make_robot_name(self, robot_id: int) -> str:
        """Create a robot name from the shared prefix and numeric id."""
        return f"{self._robot_prefix}{robot_id}"

    def _make_neighbor_names(self) -> List[str]:
        """Create neighbor names from prefix, robot id, and total robot count."""
        return [
            self._make_robot_name(idx)
            for idx in range(self._total_robots)
            if idx != self._robot_id
        ]

    def _state_callback(self, agent_name: str, msg: LogDataGeneric) -> None:
        """Update cached state from the custom Crazyswarm2 state log.

        The expected ``values`` order is configured in ``crazyflies.yaml``:
        ``x, y, z, vx, vy, vz``.
        """
        if len(msg.values) < 6:
            self.get_logger().warn(
                f"Ignoring malformed state log for {agent_name}: expected 6 values."
            )
            return

        robot = self._states.get(agent_name)
        if robot is None:
            robot = Robot(name=agent_name, radius=float(self._robot_radius))
            self._states[agent_name] = robot

        robot.update_state(
            x=msg.values[0],
            y=msg.values[1],
            z=msg.values[2],
            vx=msg.values[3],
            vy=msg.values[4],
            vz=msg.values[5],
        )

    def _control_step(self) -> None:
        """Run one local control step for this Crazyflie.

        This builds constraint terms from the active CBF constraints, solves the
        local acceleration QP, and sends the result through
        ``AccelerationCommander``.
        """
        ego = self._states.get(self._robot_name)
        if ego is None:
            self.get_logger().debug("Waiting for ego state.")
            return

        neighbors = {
            name: self._states[name]
            for name in self._neighbor_names
            if name in self._states
        }

        a_des = self._compute_acceleration(ego, neighbors)
        self._commander.send_accel(ego.position, ego.velocity, a_des)

    def _compute_acceleration(
        self,
        ego: Robot,
        neighbors: Dict[str, Robot],
    ) -> np.ndarray:
        """Compute the QP-filtered acceleration command."""

        position_ref = self._opposite_circle_goal()
        kp = 1.0
        kd = 1.4

        a_ref = kp * (position_ref - ego.position) - kd * ego.velocity
        a_ref = self._clip_norm(a_ref, self._max_acceleration)

        self._optimizer.reset()
        self._optimizer.set_objective(a_ref)

        neighbor_states = list(neighbors.values())
        for constraint in self._constraints:
            for term in constraint.compute_terms(ego, neighbor_states):
                self._optimizer.add_linear_constraint(
                    term.a_row,
                    term.b,
                    slack_label=term.slack_label,
                    slack_weight=term.slack_weight,
                )

        status, ax, ay = self._optimizer.solve()
        if status != SolveStatus.OPTIMAL:
            self.get_logger().warn(
                f"Acceleration QP failed with status {status.value}; braking."
            )
            return self._clip_norm(-2.0 * ego.velocity, self._max_acceleration)

        return np.array([ax, ay], dtype=float)

    def _opposite_circle_goal(self) -> np.ndarray:
        """Return this robot's opposite point on the scenario circle."""
        if self._total_robots <= 0:
            return np.zeros(2)

        theta = 2.0 * np.pi * self._robot_id / self._total_robots
        goal_theta = theta + np.pi
        return self._scenario_radius * np.array(
            [np.cos(goal_theta), np.sin(goal_theta)],
            dtype=float,
        )

    @staticmethod
    def _clip_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
        """Clip a vector to ``max_norm`` while preserving direction."""
        norm = np.linalg.norm(vector)
        if norm <= max_norm or norm == 0.0:
            return vector

        return vector * (max_norm / norm)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = CbfAgentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
