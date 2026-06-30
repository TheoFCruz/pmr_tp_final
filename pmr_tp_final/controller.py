"""Basic ROS 2 node skeleton for Crazyflie CBF/QP control."""

from typing import Optional, Sequence

import rclpy
from rclpy.node import Node


class CbfControllerNode(Node):
    """Minimal controller node with a periodic control callback."""

    def __init__(self) -> None:
        super().__init__("cbf_controller")

        self._control_period = 0.05
        self._timer = self.create_timer(self._control_period, self._control_step)

        self.get_logger().info("CBF controller node started.")

    def _control_step(self) -> None:
        """Periodic control loop placeholder.

        Later this should:
        1. Read/update Crazyflie states.
        2. Build CBF constraints using ``constraints.py``.
        3. Solve the QP using ``optimizer.py``.
        4. Send velocity commands to the Crazyflies.
        """
        pass


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = CbfControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
