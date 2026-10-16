"""Experiment attempt model representing a single run (1, 2, 3, etc.)."""

from typing import Optional
from pathlib import Path
from .log_data import LogData


class ExperimentAttempt:
    """
    Represents a single attempt of an experiment.
    
    Attributes:
        attempt_number: The attempt number (1, 2, 3, etc.)
        path: Path to the attempt directory
        log_data: Parsed log data from the JSONL file
    """
    
    def __init__(self, attempt_number: int, path: Path):
        """
        Initialize an experiment attempt.
        
        Args:
            attempt_number: The attempt number
            path: Path to the attempt directory
        """
        self.attempt_number = attempt_number
        self.path = path
        self.log_data: Optional[LogData] = None
    
    @property
    def logs_dir(self) -> Path:
        """Get the logs directory path."""
        return self.path / "logs"
    
    @property
    def bags_dir(self) -> Path:
        """Get the bags directory path."""
        return self.path / "bags"
    
    def has_logs(self) -> bool:
        """Check if logs directory exists and contains JSONL files."""
        if not self.logs_dir.exists():
            return False
        return any(self.logs_dir.glob("*.jsonl"))
    
    def get_log_file(self) -> Optional[Path]:
        """
        Get the first JSONL log file in the logs directory.
        
        Returns:
            Path to the log file or None if not found
        """
        if not self.has_logs():
            return None
        jsonl_files = list(self.logs_dir.glob("*.jsonl"))
        return jsonl_files[0] if jsonl_files else None
    
    def __repr__(self) -> str:
        return f"ExperimentAttempt(number={self.attempt_number}, path='{self.path}')"
