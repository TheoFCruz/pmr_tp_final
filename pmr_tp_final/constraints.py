"""Linear constraint terms for the CBF/QP controller."""

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from pmr_tp_final.robot_model import Robot

logger = logging.getLogger(__name__)


@dataclass
class ConstraintTerm:
    """One linear optimizer constraint of the form ``a_row @ a >= b``.

    ``slack_label`` should be left as ``None`` for hard constraints, such as
    safety CBFs. Relaxed constraints, such as VO-CBF terms, can set a slack
    label and corresponding quadratic penalty weight.
    """

    a_row: np.ndarray
    b: float
    slack_label: Optional[str] = None
    slack_weight: float = 1.0


class BaseConstraint(ABC):
    """Abstract interface for constraints that produce linear QP rows."""

    @property
    @abstractmethod
    def requires_slack(self) -> bool:
        """Whether this constraint uses a slack variable in the optimizer."""

    @property
    def slack_label(self) -> Optional[str]:
        """Slack variable name when ``requires_slack`` is True."""
        return None

    @abstractmethod
    def compute_terms(
        self,
        ego: Robot,
        neighbors: List[Robot],
    ) -> List[ConstraintTerm]:
        """Return constraints such that ``term.a_row @ a >= term.b``."""


class SafetyCbfConstraint(BaseConstraint):
    """Hard pairwise safety-CBF constraint.

    The neighbor is modeled as keeping constant velocity over the current QP
    step, so only the ego planar acceleration appears in each linear row.
    """

    def __init__(
        self,
        safety_margin: float,
        max_acceleration: float,
        alpha: float,
        min_distance: float = 1e-6,
    ) -> None:
        if max_acceleration <= 0.0:
            raise ValueError("max_acceleration must be positive.")
        if min_distance <= 0.0:
            raise ValueError("min_distance must be positive.")

        self.safety_margin = float(safety_margin)
        self.max_acceleration = float(max_acceleration)
        self.alpha = float(alpha)
        self.min_distance = float(min_distance)

    @property
    def requires_slack(self) -> bool:
        """Safety constraints are hard constraints and never use slack."""
        return False

    def compute_terms(
        self,
        ego: Robot,
        neighbors: List[Robot],
    ) -> List[ConstraintTerm]:
        """Return hard safety-CBF terms for all neighbors."""
        terms: List[ConstraintTerm] = []

        for neighbor in neighbors:
            term = self._compute_pair_term(ego, neighbor)
            if term is not None:
                terms.append(term)

        return terms

    def _compute_pair_term(
        self,
        ego: Robot,
        neighbor: Robot,
    ) -> Optional[ConstraintTerm]:
        """Compute one pairwise term ``a_row @ a_i >= b`` if needed."""
        p_ij = neighbor.position - ego.position
        v_ij = neighbor.velocity - ego.velocity
        distance = float(np.linalg.norm(p_ij))

        if distance < self.min_distance:
            logger.warning(
                "Skipping safety CBF for %s/%s: relative distance is too small.",
                ego.name or "ego",
                neighbor.name or "neighbor",
            )
            return None

        p_hat = p_ij / distance
        radial_speed = float(np.dot(v_ij, p_hat))
        closing_speed = min(0.0, radial_speed)

        combined_radius = ego.radius + neighbor.radius
        clearance = distance - combined_radius
        h = (
            clearance
            - self.safety_margin
            - closing_speed**2 / (2.0 * self.max_acceleration)
        )

        tangential_speed_sq = max(
            0.0,
            float(np.dot(v_ij, v_ij)) - radial_speed**2,
        )
        p_hat_dot_term = tangential_speed_sq / distance

        a_row = (closing_speed / self.max_acceleration) * p_hat
        b = (
            -radial_speed
            + (closing_speed / self.max_acceleration) * p_hat_dot_term
            - self.alpha * h
        )

        return ConstraintTerm(a_row=a_row, b=float(b))
