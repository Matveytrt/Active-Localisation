"""Parser for directory structure of gathered data."""

from pathlib import Path
from typing import List, Dict, Tuple
from ..models.dataset_root import DatasetRoot
from ..models.marker_size_group import MarkerSizeGroup
from ..models.marker_type_group import MarkerTypeGroup
from ..models.experiment import Experiment
from ..models.experiment_attempt import ExperimentAttempt
from .log_parser import LogParser


class PathParser:
    """Parser for the directory structure of gathered experiment data."""
    
    def __init__(self, log_parser: LogParser = None):
        """
        Initialize the path parser.
        
        Args:
            log_parser: LogParser instance for parsing JSONL files
        """
        self.log_parser = log_parser or LogParser()
    
    @staticmethod
    def parse_marker_type_name(directory_name: str) -> Tuple[str, str]:
        """
        Parse marker type directory name into class and variant.
        
        Examples:
            "HD17_vert" -> ("HD17", "vert")
            "HD17" -> ("HD17", "normal")
            "normal" -> ("normal", "normal")
        
        Args:
            directory_name: Directory name to parse
            
        Returns:
            Tuple of (marker_class, variant)
        """
        if '_' in directory_name:
            parts = directory_name.split('_', 1)
            return parts[0], parts[1]
        else:
            return directory_name, "normal"
    
    def parse_dataset(
        self,
        root_path: Path,
        size_configs: List[Dict[str, str]],
        load_logs: bool = True
    ) -> DatasetRoot:
        """
        Parse the entire dataset structure.
        
        Args:
            root_path: Path to gathered_data directory
            size_configs: List of size configurations from config
                         [{"name": "m40", "display": "40x40 мм"}, ...]
            load_logs: Whether to load and parse log files
            
        Returns:
            DatasetRoot object with the full hierarchy
        """
        dataset = DatasetRoot(root_path)
        
        for size_config in size_configs:
            size_code = size_config['name']
            display_name = size_config['display']
            size_path = root_path / size_code
            
            if not size_path.exists():
                print(f"Warning: Size directory not found: {size_path}")
                continue
            
            size_group = MarkerSizeGroup(size_code, display_name, size_path)
            
            # Iterate through marker type directories
            for marker_type_dir in size_path.iterdir():
                if not marker_type_dir.is_dir():
                    continue
                
                marker_class, variant = self.parse_marker_type_name(marker_type_dir.name)
                marker_type = MarkerTypeGroup(marker_class, variant, marker_type_dir)
                
                # Iterate through experiment directories
                for experiment_dir in marker_type_dir.iterdir():
                    if not experiment_dir.is_dir():
                        continue
                    
                    experiment = Experiment(experiment_dir.name, experiment_dir)
                    
                    # Iterate through attempt directories (1, 2, 3, etc.)
                    for attempt_dir in experiment_dir.iterdir():
                        if not attempt_dir.is_dir():
                            continue
                        
                        # Check if directory name is a number
                        try:
                            attempt_number = int(attempt_dir.name)
                        except ValueError:
                            continue  # Skip non-numeric directories
                        
                        attempt = ExperimentAttempt(attempt_number, attempt_dir)
                        
                        # Load log data if requested
                        if load_logs and attempt.has_logs():
                            log_file = attempt.get_log_file()
                            if log_file:
                                attempt.log_data = self.log_parser.parse(log_file)
                        
                        experiment.add_attempt(attempt)
                    
                    marker_type.add_experiment(experiment)
                
                size_group.add_marker_type(marker_type)
            
            dataset.add_marker_size(size_group)
        
        return dataset
    
    @staticmethod
    def print_dataset_summary(dataset: DatasetRoot) -> None:
        """
        Print a summary of the parsed dataset.
        
        Args:
            dataset: DatasetRoot to summarize
        """
        print(f"\n{'='*60}")
        print(f"Dataset Summary: {dataset.root_path}")
        print(f"{'='*60}")
        
        for size_code in dataset.get_size_codes():
            size_group = dataset.get_marker_size(size_code)
            print(f"\n📏 {size_group.size_code} ({size_group.display_name})")
            
            for marker_type_name in size_group.get_marker_type_names():
                marker_type = size_group.get_marker_type(marker_type_name)
                print(f"  🏷️  {marker_type.full_name}")
                
                for exp_name in marker_type.get_experiment_names():
                    experiment = marker_type.get_experiment(exp_name)
                    print(f"    🧪 {exp_name}")
                    print(f"       Attempts: {len(experiment.attempts)}")
                    
                    for attempt in experiment.attempts:
                        has_data = "✓" if attempt.log_data else "✗"
                        frames = len(attempt.log_data.frames) if attempt.log_data else 0
                        print(f"         #{attempt.attempt_number}: {has_data} "
                              f"({frames} frames)")
        
        print(f"\n{'='*60}")
        print(f"Total experiments: {len(dataset.get_all_experiments())}")
        print(f"{'='*60}\n")
