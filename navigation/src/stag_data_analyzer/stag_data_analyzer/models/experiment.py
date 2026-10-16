"""Experiment model representing a set of attempts for one experiment type."""

from typing import List, Optional
from pathlib import Path
from .experiment_attempt import ExperimentAttempt
from .statistics import Statistics


class Experiment:
    """
    Represents an experiment with multiple attempts.
    
    Attributes:
        name: Experiment name (e.g., "light_diskr_center_forward")
        path: Path to the experiment directory
        attempts: List of experiment attempts
        aggregated_stats: Aggregated statistics across all attempts
    """
    
    def __init__(self, name: str, path: Path):
        """
        Initialize an experiment.
        
        Args:
            name: Experiment name
            path: Path to the experiment directory
        """
        self.name = name
        self.path = path
        self.attempts: List[ExperimentAttempt] = []
        self.aggregated_stats: Optional[Statistics] = None
    
    def add_attempt(self, attempt: ExperimentAttempt) -> None:
        """Add an experiment attempt."""
        self.attempts.append(attempt)
        # Sort attempts by number
        self.attempts.sort(key=lambda a: a.attempt_number)
    
    def get_attempt(self, attempt_number: int) -> Optional[ExperimentAttempt]:
        """Get a specific attempt by number."""
        for attempt in self.attempts:
            if attempt.attempt_number == attempt_number:
                return attempt
        return None
    
    def has_data(self) -> bool:
        """Check if experiment has any attempts with log data."""
        return any(attempt.has_logs() for attempt in self.attempts)
    
    def __repr__(self) -> str:
        return (f"Experiment(name='{self.name}', attempts={len(self.attempts)}, "
                f"path='{self.path}')")
