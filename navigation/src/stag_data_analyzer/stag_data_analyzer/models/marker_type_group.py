"""Marker type group model (e.g., HD17_vert, normal)."""

from typing import List, Dict
from pathlib import Path
from .experiment import Experiment


class MarkerTypeGroup:
    """
    Represents a marker type group (e.g., HD17 with variant 'vert').
    
    Attributes:
        marker_class: Marker class name (e.g., "HD17")
        variant: Marker variant (e.g., "vert", "horiz", or "normal")
        path: Path to the marker type directory
        experiments: Dictionary of experiments by name
    """
    
    def __init__(self, marker_class: str, variant: str, path: Path):
        """
        Initialize a marker type group.
        
        Args:
            marker_class: Marker class (e.g., "HD17")
            variant: Marker variant (e.g., "vert" or "normal")
            path: Path to the marker type directory
        """
        self.marker_class = marker_class
        self.variant = variant
        self.path = path
        self.experiments: Dict[str, Experiment] = {}
    
    @property
    def full_name(self) -> str:
        """Get the full marker type name."""
        if self.variant == "normal":
            return self.marker_class
        return f"{self.marker_class} ({self.variant})"
    
    @property
    def directory_name(self) -> str:
        """Get the directory name representation."""
        if self.variant == "normal":
            return self.marker_class
        return f"{self.marker_class}_{self.variant}"
    
    def add_experiment(self, experiment: Experiment) -> None:
        """Add an experiment to this marker type group."""
        self.experiments[experiment.name] = experiment
    
    def get_experiment(self, name: str) -> Experiment:
        """Get an experiment by name."""
        return self.experiments.get(name)
    
    def get_experiment_names(self) -> List[str]:
        """Get list of all experiment names."""
        return list(self.experiments.keys())
    
    def __repr__(self) -> str:
        return (f"MarkerTypeGroup(class='{self.marker_class}', variant='{self.variant}', "
                f"experiments={len(self.experiments)})")
