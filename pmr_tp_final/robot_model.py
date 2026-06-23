"""Pure Python robot state used by the controller and simulation."""

import math


class Robot:
    """
    Represents a robot on a 2D plane with an orientation.

    Pure Python representation of a robot.
    Safe for pickling, multiprocessing, and internal math.
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def step(self, sampling_time: float, vx: float, vy: float, w: float):
        self.x += vx * sampling_time
        self.y += vy * sampling_time
        self.theta += w * sampling_time
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def __repr__(self) -> str:
        return f"Robot(x={self.x:.2f}, y={self.y:.2f}, theta={math.degrees(self.theta):.2f}°)"
