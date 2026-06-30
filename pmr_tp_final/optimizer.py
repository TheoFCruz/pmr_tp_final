"""Numerical optimizer abstractions for acceleration-based CBF/QP control."""

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
    """Abstract interface for planar acceleration QP solvers."""

    @abstractmethod
    def initialize_model(self, max_acceleration: float) -> None:
        """Set up acceleration decision variables and solver model."""

    @abstractmethod
    def reset(self) -> None:
        """Clear constraints from the previous control cycle."""

    @abstractmethod
    def set_objective(
        self,
        ref_accel: np.ndarray,
    ) -> None:
        """Configure the quadratic cost around a reference acceleration."""

    @abstractmethod
    def add_linear_constraint(
        self,
        a_row: np.ndarray,
        b: float,
        slack_label: Optional[str] = None,
        slack_weight: float = 1.0,
    ) -> None:
        """Add a linear constraint of the form a_row @ a >= b.

        When ``slack_label`` is provided, the constraint is softened with a
        non-negative slack variable. ``slack_weight`` controls that slack's
        quadratic penalty in the objective.
        """

    @abstractmethod
    def solve(self) -> Tuple[SolveStatus, float, float]:
        """Execute the solver and return status plus optimal acceleration."""


class AccelOptimizer(BaseOptimizer):
    """Gurobi-backed QP optimizer for planar acceleration commands."""

    def __init__(self, time_limit: float = 0.05):
        """Create the Gurobi environment used by this optimizer."""
        self._time_limit = time_limit
        self._env: Optional[gp.Env] = None
        self._model: Optional[gp.Model] = None
        self._vars: Dict[str, gp.Var] = {}
        self._ref_accel: Optional[np.ndarray] = None
        self._slack_weights: Dict[str, float] = {}

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

    def initialize_model(self, max_acceleration: float) -> None:
        """Create ``ax`` and ``ay`` decision variables with symmetric bounds."""
        if self._env is None:
            raise RuntimeError("Gurobi environment is not initialized.")

        model = gp.Model("acceleration_control_qp", env=self._env)
        model.setParam("TimeLimit", self._time_limit)

        ax = model.addVar(
            lb=-max_acceleration,
            ub=max_acceleration,
            name="ax",
        )
        ay = model.addVar(
            lb=-max_acceleration,
            ub=max_acceleration,
            name="ay",
        )

        self._model = model
        self._vars = {
            "ax": ax,
            "ay": ay,
        }
        self._slack_weights = {}

    def reset(self) -> None:
        """Remove all constraints from the previous control iteration."""
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        self._model.remove(self._model.getConstrs())
        self._model.update()

    def set_objective(
        self,
        ref_accel: np.ndarray,
    ) -> None:
        """Minimize distance to ``ref_accel`` plus weighted slack penalties."""
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        ref_accel = np.asarray(ref_accel, dtype=float)
        if ref_accel.size < 2:
            raise ValueError("ref_accel must contain at least [ax_ref, ay_ref].")

        self._ref_accel = ref_accel
        self._set_objective()

    def _set_objective(self) -> None:
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")
        if self._ref_accel is None:
            return

        ax = self._vars["ax"]
        ay = self._vars["ay"]
        objective = (
            (ax - self._ref_accel[0]) * (ax - self._ref_accel[0])
            + (ay - self._ref_accel[1]) * (ay - self._ref_accel[1])
        )
        for label, var in self._vars.items():
            if label not in ("ax", "ay"):
                objective += self._slack_weights.get(label, 1.0) * var * var

        self._model.setObjective(objective, GRB.MINIMIZE)

    def add_linear_constraint(
        self,
        a_row: np.ndarray,
        b: float,
        slack_label: Optional[str] = None,
        slack_weight: float = 1.0,
    ) -> None:
        """Add ``a_row @ [ax, ay] >= b``, optionally softened by a slack."""
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        a_row = np.asarray(a_row, dtype=float)
        if a_row.size < 2:
            raise ValueError("a_row must contain at least two values.")

        ax = self._vars["ax"]
        ay = self._vars["ay"]
        lhs = a_row[0] * ax + a_row[1] * ay

        if slack_label is not None:
            if slack_weight < 0.0:
                raise ValueError("slack_weight must be non-negative.")
            self._slack_weights[slack_label] = slack_weight
            slack_var = self._get_or_create_slack(slack_label)
            self._set_objective()
            self._model.addConstr(lhs >= b - slack_var)
        else:
            self._model.addConstr(lhs >= b)

    def _get_or_create_slack(self, slack_label: str) -> gp.Var:
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        if slack_label not in self._vars:
            self._vars[slack_label] = self._model.addVar(lb=0.0, name=slack_label)
            self._model.update()

        return self._vars[slack_label]

    def solve(self) -> Tuple[SolveStatus, float, float]:
        """Run the QP and return ``(status, ax, ay)``."""
        if self._model is None:
            raise RuntimeError("Optimizer model is not initialized.")

        try:
            self._model.optimize()
        except gp.GurobiError:
            return SolveStatus.FAILED, 0.0, 0.0

        status_map = {
            GRB.OPTIMAL: SolveStatus.OPTIMAL,
            GRB.INFEASIBLE: SolveStatus.INFEASIBLE,
            GRB.TIME_LIMIT: SolveStatus.TIME_LIMIT,
        }
        status = status_map.get(self._model.status, SolveStatus.FAILED)

        if status == SolveStatus.OPTIMAL:
            return (
                status,
                self._vars["ax"].X,
                self._vars["ay"].X,
            )

        return status, 0.0, 0.0
