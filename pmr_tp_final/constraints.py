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
    """Hard pairwise high-order distance safety-CBF constraint.

    The barrier is based directly on squared inter-agent distance:

    ``h = ||p_j - p_i||² - (r_i + r_j + safety_margin)²``.

    Since the controller input is acceleration, this uses the second-order CBF
    condition ``h_ddot + 2 alpha h_dot + alpha² h >= 0``. The neighbor is
    modeled as keeping constant velocity over the current QP step, so only the
    ego planar acceleration appears in each linear row.
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
        distance_sq = float(np.dot(p_ij, p_ij))
        distance = float(np.sqrt(distance_sq))

        if distance < self.min_distance:
            logger.warning(
                "Skipping safety CBF for %s/%s: relative distance is too small.",
                ego.name or "ego",
                neighbor.name or "neighbor",
            )
            return None

        safe_distance = ego.radius + neighbor.radius + self.safety_margin
        h = distance_sq - safe_distance**2
        h_dot = 2.0 * float(np.dot(p_ij, v_ij))

        # h_ddot = 2 ||v_ij||² - 2 p_ijᵀ a_i, assuming neighbor acceleration is
        # zero. Enforce h_ddot + 2 alpha h_dot + alpha² h >= 0 and convert to
        # optimizer form a_row @ a_i >= b.
        relative_speed_sq = float(np.dot(v_ij, v_ij))
        a_row = -2.0 * p_ij
        b = (
            -2.0 * relative_speed_sq
            - 2.0 * self.alpha * h_dot
            - self.alpha**2 * h
        )

        return ConstraintTerm(a_row=a_row, b=float(b))


class VelocitySafetyCbfConstraint(BaseConstraint):
    """Hard braking-distance safety-CBF constraint kept for comparison.

    This is the previous velocity-based safety barrier:

    ``h = d_ij - safety_margin - min(0, v_ijᵀ p_hat)² / (2 a_max)``.

    It is not used by the current controller, which instantiates
    ``SafetyCbfConstraint`` above instead.
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
        """Return hard braking-distance CBF terms for all neighbors."""
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
                "Skipping velocity safety CBF for %s/%s: relative distance is too small.",
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


class VelocityObstacleCbfConstraint(BaseConstraint):
    """Relaxed velocity-obstacle CBF constraint.

    For circular agents, the VO boundary angle is defined by the tangent from
    the ego center to the neighbor's inflated circle. With

    ``p = p_j - p_i``, ``v = v_j - v_i``, ``R = r_i + r_j``, and
    ``L = sqrt(||p||² - R²)``, an angle-based VO barrier can be written without
    explicitly choosing either tangent side as

    ``h_vo = pᵀ v + L ||v||``.

    The neighbor is modeled as constant velocity over the current QP step, so
    only the ego acceleration appears in the linear row. The returned terms use
    slack variables and are therefore soft guidance constraints, unlike the
    hard safety CBF above.
    """

    def __init__(
        self,
        alpha: float,
        slack_weight_gain: float,
        min_distance: float = 1e-6,
        min_relative_speed: float = 1e-6,
        min_time_to_collision: float = 1e-3,
        max_time_to_collision: float = 5.0,
    ) -> None:
        if min_distance <= 0.0:
            raise ValueError("min_distance must be positive.")
        if min_relative_speed <= 0.0:
            raise ValueError("min_relative_speed must be positive.")
        if min_time_to_collision <= 0.0:
            raise ValueError("min_time_to_collision must be positive.")
        if max_time_to_collision <= min_time_to_collision:
            raise ValueError("max_time_to_collision must exceed min_time_to_collision.")

        self.alpha = float(alpha)
        self.slack_weight_gain = float(slack_weight_gain)
        self.min_distance = float(min_distance)
        self.min_relative_speed = float(min_relative_speed)
        self.min_time_to_collision = float(min_time_to_collision)
        self.max_time_to_collision = float(max_time_to_collision)

    @property
    def requires_slack(self) -> bool:
        """VO constraints are relaxed with one slack variable per neighbor."""
        return True

    def compute_terms(
        self,
        ego: Robot,
        neighbors: List[Robot],
    ) -> List[ConstraintTerm]:
        """Return relaxed VO-CBF terms for approaching neighbors."""
        terms: List[ConstraintTerm] = []

        for idx, neighbor in enumerate(neighbors):
            slack_label = self._slack_label(ego, neighbor, idx)
            term = self._compute_pair_term(ego, neighbor, slack_label)
            if term is not None:
                terms.append(term)

        return terms

    def _compute_pair_term(
        self,
        ego: Robot,
        neighbor: Robot,
        slack_label: str,
    ) -> Optional[ConstraintTerm]:
        """Compute one relaxed VO-CBF term ``a_row @ a_i >= b - slack``."""
        p_ij = neighbor.position - ego.position
        v_ij = neighbor.velocity - ego.velocity
        distance = float(np.linalg.norm(p_ij))
        relative_speed = float(np.linalg.norm(v_ij))
        combined_radius = ego.radius + neighbor.radius

        if distance <= combined_radius or distance < self.min_distance:
            logger.warning(
                "Skipping VO-CBF for %s/%s: tangent is undefined.",
                ego.name or "ego",
                neighbor.name or "neighbor",
            )
            return None
        if relative_speed < self.min_relative_speed:
            return None

        p_dot_v = float(np.dot(p_ij, v_ij))
        if p_dot_v >= 0.0:
            return None

        time_weight = self._time_weight(p_ij, v_ij, combined_radius)

        # Paper notation for the simplified Eq. 23 term:
        #
        #   uᵀ (p + v_hat (pᵀ l_hat))
        #   + ||v|| (vᵀ l_hat + pᵀ l_hat_dot + ||v||)
        #
        # For circular agents, choosing either tangent side gives the same
        # scalar pᵀ l_hat = L = sqrt(||p||² - R²). This lets us avoid
        # explicitly constructing l_hat and l_hat_dot.
        l_norm = float(np.sqrt(distance**2 - combined_radius**2))
        v_norm = relative_speed
        v_hat = v_ij / v_norm

        p_dot_l_hat = l_norm
        p_dot_l_hat_dot_plus_v_dot_l_hat = p_dot_v / l_norm

        h_vo = p_dot_v + p_dot_l_hat * v_norm

        u_coefficient = p_ij + v_hat * p_dot_l_hat
        paper_constant_term = v_norm * (
            p_dot_l_hat_dot_plus_v_dot_l_hat + v_norm
        )

        # The paper expression is written for the relative acceleration u.
        # Here v_ij = v_j - v_i and the neighbor is assumed constant velocity,
        # so relative acceleration is -a_i. Convert to optimizer form:
        #
        #   a_row @ a_i >= b - slack
        a_row = -u_coefficient
        b = -paper_constant_term - self.alpha * h_vo

        slack_weight = self.slack_weight_gain / max(
            time_weight,
            self.min_time_to_collision,
        )

        return ConstraintTerm(
            a_row=a_row,
            b=float(b),
            slack_label=slack_label,
            slack_weight=float(slack_weight),
        )

    def _time_weight(
        self,
        p_ij: np.ndarray,
        v_ij: np.ndarray,
        combined_radius: float,
    ) -> float:
        """Return a finite time scale for VO slack weighting.

        If the current relative velocity intersects the collision disk, use the
        true first positive collision time. Otherwise use the time of closest
        approach, capped by ``max_time_to_collision``. This keeps the relaxed VO
        term active earlier than the exact collision-cone crossing.
        """
        time_to_collision = self._time_to_collision(
            p_ij,
            v_ij,
            combined_radius,
        )
        if time_to_collision is not None:
            return min(time_to_collision, self.max_time_to_collision)

        relative_speed_sq = float(np.dot(v_ij, v_ij))
        time_to_closest_approach = -float(np.dot(p_ij, v_ij)) / relative_speed_sq
        return min(
            max(time_to_closest_approach, self.min_time_to_collision),
            self.max_time_to_collision,
        )

    def _time_to_collision(
        self,
        p_ij: np.ndarray,
        v_ij: np.ndarray,
        combined_radius: float,
    ) -> Optional[float]:
        """Return the first positive collision time, or None if none exists."""
        a = float(np.dot(v_ij, v_ij))
        b = 2.0 * float(np.dot(p_ij, v_ij))
        c = float(np.dot(p_ij, p_ij)) - combined_radius**2

        if a < self.min_relative_speed**2:
            return None

        discriminant = b**2 - 4.0 * a * c
        if discriminant < 0.0:
            return None

        sqrt_discriminant = float(np.sqrt(discriminant))
        roots = [
            (-b - sqrt_discriminant) / (2.0 * a),
            (-b + sqrt_discriminant) / (2.0 * a),
        ]
        positive_roots = [root for root in roots if root > 0.0]
        if not positive_roots:
            return None

        return min(positive_roots)

    @staticmethod
    def _slack_label(ego: Robot, neighbor: Robot, neighbor_index: int) -> str:
        ego_name = ego.name or "ego"
        neighbor_name = neighbor.name or f"neighbor_{neighbor_index}"
        return f"vo_{ego_name}_{neighbor_name}"
