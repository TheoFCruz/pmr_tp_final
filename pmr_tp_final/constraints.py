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
    """Hard pairwise safety-CBF constraint skeleton.

    This class owns the parameters needed by the safety CBF and exposes the
    common ``BaseConstraint`` interface. The actual CBF row computation will be
    added in a later step.
    """

    def __init__(
        self,
        safety_margin: float,
        max_acceleration: float,
        alpha: float,
        min_distance: float = 1e-6,
    ) -> None:
        self.safety_margin = safety_margin
        self.max_acceleration = max_acceleration
        self.alpha = alpha
        self.min_distance = min_distance

    @property
    def requires_slack(self) -> bool:
        """Safety constraints are hard constraints and never use slack."""
        return False

    def compute_terms(
        self,
        ego: Robot,
        neighbors: List[Robot],
    ) -> List[ConstraintTerm]:
        """Return hard safety-CBF terms for all neighbors.

        TODO: implement the actual CBF formula. Until then, return no terms so
        the controller wiring can be exercised without changing behavior.
        """
        del ego, neighbors
        return []
