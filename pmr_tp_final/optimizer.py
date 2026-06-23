"""Numerical optimizer abstractions for the visibility-guard QP controller."""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict, Optional, Tuple

import gurobipy as gp
from gurobipy import GRB
import numpy as np


class SolveStatus(Enum):
    """Generic solver status codes independent of backend."""

    OPTIMAL = "optimal"
    INFEASIBLE = "infeasible"
    TIME_LIMIT = "time_limit"
    FAILED = "failed"


class BaseOptimizer(ABC):
    """Abstract interface for control-action QP solvers."""

    @abstractmethod
    def initialize_model(self, max_linear_speed: float, max_angular_speed: float) -> None:
        """Set up decision variables and solver model."""

    @abstractmethod
    def reset(self) -> None:
        """Clear constraints from the previous control cycle."""

    @abstractmethod
    def set_objective(self, role: str, P: np.ndarray, ref_vel: np.ndarray) -> None:
        """Configure the quadratic cost for the current robot role."""

    @abstractmethod
    def add_linear_constraint(
        self,
        a_row: np.ndarray,
        b: float,
        slack_label: Optional[str] = None,
    ) -> None:
        """Add a linear constraint of the form a_row @ u >= b (optionally softened)."""

    @abstractmethod
    def solve(self) -> Tuple[SolveStatus, float, float, float]:
        """Execute the solver and return status plus optimal velocities."""


class GurobiOptimizer(BaseOptimizer):
    """Gurobi-backed QP optimizer for robot velocity commands."""

    def __init__(self, time_limit: float = 0.05):
        self._time_limit = time_limit
        self._env: Optional[gp.Env] = None
        self._model: Optional[gp.Model] = None
        self._vars: Dict[str, gp.Var] = {}

        try:
            env = gp.Env(empty=True)
            env.setParam("LogToConsole", 0)
            env.start()
            self._env = env
        except gp.GurobiError as exc:
            raise RuntimeError(f"Failed to initialize Gurobi environment: {exc}") from exc

    @property
    def is_ready(self) -> bool:
        """Return True when the Gurobi environment and model are initialized."""
        return self._env is not None and self._model is not None

    def initialize_model(self, max_linear_speed: float, max_angular_speed: float) -> None:
        if self._env is None:
            raise RuntimeError("Gurobi environment is not initialized.")

        model = gp.Model("robot_control_qp", env=self._env)
        model.setParam("TimeLimit", self._time_limit)

        vx = model.addVar(
            lb=-max_linear_speed,
            ub=max_linear_speed,
            name="vx",
        )
        vy = model.addVar(
            lb=-max_linear_speed,
            ub=max_linear_speed,
            name="vy",
        )
        w = model.addVar(
            lb=-max_angular_speed,
            ub=max_angular_speed,
            name="w",
        )
        slack_follower = model.addVar(lb=0.0, name="slack_follower")
        slack_leader = model.addVar(lb=0.0, name="slack_leader")

        self._model = model
        self._vars = {
            "vx": vx,
            "vy": vy,
            "w": w,
            "slack_follower": slack_follower,
            "slack_leader": slack_leader,
        }

    def reset(self) -> None:
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        self._model.remove(self._model.getConstrs())
        self._model.update()

    def set_objective(self, role: str, P: np.ndarray, ref_vel: np.ndarray) -> None:
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        vx = self._vars["vx"]
        vy = self._vars["vy"]
        w = self._vars["w"]
        slack_follower = self._vars["slack_follower"]
        slack_leader = self._vars["slack_leader"]

        cost_control = P[0] * vx * vx + P[1] * vy * vy + P[2] * w * w

        if role == "leader":
            cost_ref = (
                P[0] * (vx - ref_vel[0]) ** 2
                + P[1] * (vy - ref_vel[1]) ** 2
                + P[2] * (w - ref_vel[2]) ** 2
            )
            objective = cost_ref + slack_follower * slack_follower
        else:
            objective = (
                cost_control
                + slack_follower * slack_follower
                + slack_leader * slack_leader
            )

        self._model.setObjective(objective, GRB.MINIMIZE)

    def add_linear_constraint(
        self,
        a_row: np.ndarray,
        b: float,
        slack_label: Optional[str] = None,
    ) -> None:
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        u = np.array([self._vars["vx"], self._vars["vy"], self._vars["w"]])

        if slack_label is not None:
            slack_var = self._vars[slack_label]
            self._model.addConstr(a_row @ u >= b - slack_var)
        else:
            self._model.addConstr(a_row @ u >= b)

    def solve(self) -> Tuple[SolveStatus, float, float, float]:
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        try:
            self._model.optimize()
        except gp.GurobiError:
            return SolveStatus.FAILED, 0.0, 0.0, 0.0

        status_map = {
            GRB.OPTIMAL: SolveStatus.OPTIMAL,
            GRB.INFEASIBLE: SolveStatus.INFEASIBLE,
            GRB.TIME_LIMIT: SolveStatus.TIME_LIMIT,
        }
        status = status_map.get(self._model.status, SolveStatus.FAILED)

        if status == SolveStatus.OPTIMAL:
            return (
                status,
                self._vars["vx"].X,
                self._vars["vy"].X,
                self._vars["w"].X,
            )

        return status, 0.0, 0.0, 0.0
