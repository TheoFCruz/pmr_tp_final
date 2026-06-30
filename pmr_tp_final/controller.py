"""Pseudo-decentralized ROS 2 node skeleton for Crazyflie CBF/QP control."""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np
import rclpy
from crazyflie_interfaces.msg import LogDataGeneric
from rclpy.node import Node

from pmr_tp_final.commander import AccelerationCommander


@dataclass
class AgentState:
    """Planar state used by each local CBF controller."""

    position: np.ndarray
    velocity: np.ndarray


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
        self.declare_parameter("control_period", 0.05)
        self.declare_parameter("fixed_z", 1.0)
        self.declare_parameter("max_velocity", 1.0)
        self.declare_parameter("max_acceleration", 1.0)

        self._robot_id = self.get_parameter("robot_id").value
        self._robot_prefix = self.get_parameter("robot_prefix").value
        self._total_robots = self.get_parameter("total_robots").value
        self._control_period = self.get_parameter("control_period").value
        self._robot_name = self._make_robot_name(self._robot_id)
        self._neighbor_names = self._make_neighbor_names()

        self._states: Dict[str, AgentState] = {}
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

        self._states[agent_name] = AgentState(
            position=np.array([msg.values[0], msg.values[1]], dtype=float),
            velocity=np.array([msg.values[3], msg.values[4]], dtype=float),
        )

    def _control_step(self) -> None:
        """Run one local control step for this Crazyflie.

        Later this should:
        1. Build CBF constraints from ego and neighbor states.
        2. Solve this agent's QP using ``optimizer.py``.
        3. Send the resulting acceleration through ``AccelerationCommander``.
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
        ego: AgentState,
        neighbors: Dict[str, AgentState],
    ) -> np.ndarray:
        """Placeholder for the local CBF/VO QP.

        This method should eventually use ``constraints.py`` and
        ``optimizer.py`` to compute this agent's acceleration command.
        """
        del ego, neighbors
        return np.zeros(2)


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
