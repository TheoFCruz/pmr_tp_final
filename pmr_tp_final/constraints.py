"""Linear constraint terms for the visibility-guard QP controller."""

import logging
from abc import ABC, abstractmethod
from typing import Any, List, Optional, Tuple

import numpy as np

from pmr_tp_final.robot_model import Robot

logger = logging.getLogger(__name__)

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
        occupancy_map: Any,
        map_info: Any,
    ) -> List[Tuple[np.ndarray, float]]:
        """Return (A_row, b) pairs such that A_row @ u >= b."""


