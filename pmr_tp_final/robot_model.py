"""Pure Python robot state used by the controller and constraints."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Robot:
    """Robot state used by the pseudo-decentralized CBF controller.

    The controller and constraints operate on planar position/velocity, while
    the ROS state log also provides altitude and vertical velocity. Keep all of
    those values here so constraints do not need to depend on ROS messages or
    controller-local state classes.
    """

    name: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    theta: float = 0.0
    radius: float = 0.0

    @property
    def position(self) -> np.ndarray:
        """Planar position ``[x, y]`` used by the QP constraints."""
        return np.array([self.x, self.y], dtype=float)

    @property
    def velocity(self) -> np.ndarray:
        """Planar velocity ``[vx, vy]`` used by the QP constraints."""
        return np.array([self.vx, self.vy], dtype=float)

    @property
    def position_3d(self) -> np.ndarray:
        """Full logged position ``[x, y, z]``."""
        return np.array([self.x, self.y, self.z], dtype=float)

    @property
    def velocity_3d(self) -> np.ndarray:
        """Full logged velocity ``[vx, vy, vz]``."""
        return np.array([self.vx, self.vy, self.vz], dtype=float)

    def update_state(
        self,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        """Update measured position and velocity from a state log."""
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.vx = float(vx)
        self.vy = float(vy)
        self.vz = float(vz)

    def __repr__(self) -> str:
        name = f"name={self.name}, " if self.name else ""
        return (
            f"Robot({name}x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, "
            f"vx={self.vx:.2f}, vy={self.vy:.2f}, vz={self.vz:.2f}, "
            f"theta={math.degrees(self.theta):.2f}°, radius={self.radius:.2f})"
        )
