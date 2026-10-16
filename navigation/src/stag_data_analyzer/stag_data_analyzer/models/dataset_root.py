"""Root dataset model managing all marker sizes and experiments."""

from typing import Dict, List, Optional
from pathlib import Path
from .marker_size_group import MarkerSizeGroup
from .experiment import Experiment


class DatasetRoot:
    """
    Root dataset managing all marker size groups.
    
    Attributes:
        root_path: Path to the gathered_data directory
        marker_sizes: Dictionary of marker size groups by size code
    """
    
    def __init__(self, root_path: Path):
        """
        Initialize the dataset root.
        
        Args:
            root_path: Path to the gathered_data directory
        """
        self.root_path = Path(root_path)
        self.marker_sizes: Dict[str, MarkerSizeGroup] = {}
    
    def add_marker_size(self, size_group: MarkerSizeGroup) -> None:
        """Add a marker size group."""
        self.marker_sizes[size_group.size_code] = size_group
    
    def get_marker_size(self, size_code: str) -> Optional[MarkerSizeGroup]:
        """Get a marker size group by code."""
        return self.marker_sizes.get(size_code)
    
    def get_size_codes(self) -> List[str]:
        """Get list of all size codes."""
        return list(self.marker_sizes.keys())
    
    def get_all_experiments(self) -> List[Experiment]:
        """
        Get all experiments from all marker sizes and types.
        
        Returns:
            List of all Experiment objects
        """
        experiments = []
        for size_group in self.marker_sizes.values():
            for marker_type in size_group.marker_types.values():
                experiments.extend(marker_type.experiments.values())
        return experiments
    
    def find_experiments_by_name(self, experiment_name: str) -> List[Experiment]:
        """
        Find all experiments with a specific name across all sizes and types.
        Useful for comparing the same experiment across different marker sizes.
        
        Args:
            experiment_name: Name of the experiment to find
            
        Returns:
            List of matching experiments
        """
        matching = []
        for size_group in self.marker_sizes.values():
            for marker_type in size_group.marker_types.values():
                if experiment_name in marker_type.experiments:
                    matching.append(marker_type.experiments[experiment_name])
        return matching
    
    def __repr__(self) -> str:
        total_experiments = len(self.get_all_experiments())
        return (f"DatasetRoot(path='{self.root_path}', "
                f"sizes={len(self.marker_sizes)}, "
                f"total_experiments={total_experiments})")
