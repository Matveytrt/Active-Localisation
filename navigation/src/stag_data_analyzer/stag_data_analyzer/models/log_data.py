"""Log data model representing parsed JSONL log file."""

from typing import List
from dataclasses import dataclass


@dataclass
class FrameData:
    """Single frame detection data."""
    timestamp: float
    frame_id: int
    marker_ids: List[int]


@dataclass
class LogData:
    """
    Parsed log data from a JSONL file.
    
    Attributes:
        file_path: Path to the JSONL file
        frames: List of frame detection data
        log_version: Version from metadata
        created_at: Creation timestamp from metadata
        total_frames: Total frames from closing metadata
    """
    
    file_path: str
    frames: List[FrameData]
    log_version: str = "1.0"
    created_at: str = ""
    total_frames: int = 0
    
    def __repr__(self) -> str:
        return f"LogData(file='{self.file_path}', frames={len(self.frames)})"
