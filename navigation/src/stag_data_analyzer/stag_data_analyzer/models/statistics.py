"""Statistics data model for aggregated experiment results."""

from typing import Dict
from dataclasses import dataclass, field


@dataclass
class Statistics:
    """
    Aggregated statistics for an experiment.
    
    Attributes:
        marker_detections: Dict mapping marker_id to detection count
        fps_min: Minimum FPS observed
        fps_max: Maximum FPS observed
        fps_avg: Median FPS (name kept for compatibility)
        simultaneous_markers_min: Minimum markers detected simultaneously
        simultaneous_markers_max: Maximum markers detected simultaneously
        simultaneous_markers_avg: Average markers detected simultaneously
        total_frames: Total number of frames processed
    """
    
    marker_detections: Dict[int, int] = field(default_factory=dict)
    fps_min: float = 0.0
    fps_max: float = 0.0
    fps_avg: float = 0.0
    simultaneous_markers_min: int = 0
    simultaneous_markers_max: int = 0
    simultaneous_markers_avg: float = 0.0
    total_frames: int = 0
    
    def __repr__(self) -> str:
        return (f"Statistics(fps={self.fps_min:.1f}-{self.fps_max:.1f} "
                f"(avg={self.fps_avg:.1f}), "
                f"simultaneous_markers={self.simultaneous_markers_min}-"
                f"{self.simultaneous_markers_max} "
                f"(avg={self.simultaneous_markers_avg:.1f}), "
                f"frames={self.total_frames})")
