#!/usr/bin/env python3
"""
Path Generator for STag Experiments

Generates file names and directory paths for experiment outputs.
Handles marker IDs with a suffix (HG/HD: library name vs full marker ID).
"""

from pathlib import Path
from typing import Tuple


class PathGenerator:
    """
    Generates standardized paths and filenames for experiments.
    
    Format:
    - Filename: {marker}_{experiment}_{gait}_{attempt}
    - Directory: {base_path}/{marker}/{experiment}_{gait}/{attempt}/
    - Inside each attempt: bags/ and logs/ subdirectories
    """
    
    def __init__(self, base_path: str, bags_subdir: str = "bags", logs_subdir: str = "logs"):
        """
        Initialize path generator.
        
        Args:
            base_path: Base directory for all outputs
            bags_subdir: Subdirectory name for bag files (default: "bags")
            logs_subdir: Subdirectory name for log files (default: "logs")
        """
        self.base_path = Path(base_path)
        self.bags_subdir = bags_subdir
        self.logs_subdir = logs_subdir
    
    def extract_library_name(self, marker: str) -> str:
        """
        Extract library name from marker ID.
        
        For HG/HD markers with a suffix after an underscore (e.g., HG17_255-000,
        HD17_vert), return only the base library name (e.g., HG17, HD17).
        For other markers, return as-is.
        
        Args:
            marker: Full marker ID (e.g., "HG17_255-000", "HD17_vert", "HC17")
        
        Returns:
            Library name (e.g., "HG17", "HD17", "HC17")
        """
        # HG/HD marker with a suffix after an underscore
        if (marker.startswith("HG") or marker.startswith("HD") ) and "_" in marker:
            # Extract everything before the first underscore
            return marker.split("_")[0]
        return marker
    
    def generate_filename(self, marker: str, experiment: str, gait: str, attempt: int) -> str:
        """
        Generate filename for bags and logs.
        
        Format: {marker}_{experiment}_{gait}_{attempt}
        
        Args:
            marker: Full marker ID (e.g., "HG17_255-000")
            experiment: Experiment type (e.g., "light")
            gait: Gait type (e.g., "slow")
            attempt: Attempt number (1-indexed)
        
        Returns:
            Filename without extension (e.g., "HG17_255-000_light_slow_1")
        """
        return f"{marker}_{experiment}_{gait}_{attempt}"
    
    def generate_directory_path(self, marker: str, experiment: str, gait: str, attempt: int) -> Path:
        """
        Generate directory path for this experiment attempt.
        
        Format: {base_path}/{marker}/{experiment}_{gait}/{attempt}/
        
        Args:
            marker: Full marker ID
            experiment: Experiment type
            gait: Gait type
            attempt: Attempt number (1-indexed)
        
        Returns:
            Path object for the attempt directory
        """
        return self.base_path / marker / f"{experiment}_{gait}" / str(attempt)
    
    def generate_full_paths(
        self, 
        marker: str, 
        experiment: str, 
        gait: str, 
        attempt: int
    ) -> Tuple[Path, Path, str, str]:
        """
        Generate all paths needed for an experiment.
        
        Args:
            marker: Full marker ID
            experiment: Experiment type
            gait: Gait type
            attempt: Attempt number (1-indexed)
        
        Returns:
            Tuple of (bags_dir, logs_dir, bag_filename, log_filename)
        """
        # Get base attempt directory
        attempt_dir = self.generate_directory_path(marker, experiment, gait, attempt)
        
        # Generate subdirectories
        bags_dir = attempt_dir / self.bags_subdir
        logs_dir = attempt_dir / self.logs_subdir
        
        # Generate filenames (with extensions)
        base_filename = self.generate_filename(marker, experiment, gait, attempt)
        bag_filename = f"{base_filename}.db3"  # rosbag2 sqlite3 format
        log_filename = f"{base_filename}.jsonl"
        
        return bags_dir, logs_dir, bag_filename, log_filename
    
    def create_directories(self, bags_dir: Path, logs_dir: Path) -> None:
        """
        Create bag and log directories if they don't exist.
        
        Args:
            bags_dir: Path to bags directory
            logs_dir: Path to logs directory
        """
        bags_dir.mkdir(parents=True, exist_ok=True)
        logs_dir.mkdir(parents=True, exist_ok=True)
    
    def get_absolute_paths(
        self,
        marker: str,
        experiment: str,
        gait: str,
        attempt: int,
        create_dirs: bool = True
    ) -> dict:
        """
        Get all absolute paths and filenames for an experiment.
        
        Args:
            marker: Full marker ID
            experiment: Experiment type
            gait: Gait type
            attempt: Attempt number (1-indexed)
            create_dirs: Whether to create directories (default: True)
        
        Returns:
            Dictionary with keys:
            - library_name: Library name for detector (e.g., "HG17")
            - bags_dir: Absolute path to bags directory
            - logs_dir: Absolute path to logs directory
            - bag_file: Absolute path to bag file
            - log_file: Absolute path to log file
            - bag_filename: Just the filename (with extension)
            - log_filename: Just the filename (with extension)
        """
        bags_dir, logs_dir, bag_filename, log_filename = self.generate_full_paths(
            marker, experiment, gait, attempt
        )
        
        if create_dirs:
            self.create_directories(bags_dir, logs_dir)
        
        return {
            "library_name": self.extract_library_name(marker),
            "marker_full": marker,
            "bags_dir": str(bags_dir.resolve()),
            "logs_dir": str(logs_dir.resolve()),
            "bag_file": str((bags_dir / bag_filename).resolve()),
            "log_file": str((logs_dir / log_filename).resolve()),
            "bag_filename": bag_filename,
            "log_filename": log_filename,
        }
