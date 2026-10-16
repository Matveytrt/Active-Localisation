"""Marker size group model (e.g., m40, m80)."""

from typing import Dict
from pathlib import Path
from .marker_type_group import MarkerTypeGroup


class MarkerSizeGroup:
    """
    Represents a marker size group (e.g., m40 = 40x40 mm).
    
    Attributes:
        size_code: Size code (e.g., "m40")
        display_name: Human-readable size (e.g., "40x40 мм")
        path: Path to the size directory
        marker_types: Dictionary of marker type groups
    """
    
    def __init__(self, size_code: str, display_name: str, path: Path):
        """
        Initialize a marker size group.
        
        Args:
            size_code: Size code (e.g., "m40")
            display_name: Display name (e.g., "40x40 мм")
            path: Path to the size directory
        """
        self.size_code = size_code
        self.display_name = display_name
        self.path = path
        self.marker_types: Dict[str, MarkerTypeGroup] = {}
    
    def add_marker_type(self, marker_type: MarkerTypeGroup) -> None:
        """Add a marker type group."""
        key = marker_type.directory_name
        self.marker_types[key] = marker_type
    
    def get_marker_type(self, directory_name: str) -> MarkerTypeGroup:
        """Get a marker type by directory name."""
        return self.marker_types.get(directory_name)
    
    def get_marker_type_names(self) -> list:
        """Get list of all marker type directory names."""
        return list(self.marker_types.keys())
    
    def __repr__(self) -> str:
        return (f"MarkerSizeGroup(size='{self.size_code}', "
                f"display='{self.display_name}', "
                f"marker_types={len(self.marker_types)})")
