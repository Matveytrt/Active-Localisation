"""Marker size comparator for comparing same markers in different sizes."""

from typing import List, Dict, Optional
from pathlib import Path
import numpy as np

from stag_data_analyzer.models.dataset_root import DatasetRoot
from stag_data_analyzer.models.experiment import Experiment
from stag_data_analyzer.models.statistics import Statistics
from stag_data_analyzer.models.comparison_metrics import ComparisonMetrics
from stag_data_analyzer.analyzers.statistics_calculator import StatisticsCalculator


class MarkerSizeComparator:
    """Comparator for marker size analysis."""
    
    def __init__(self, dataset: DatasetRoot):
        """
        Initialize comparator.
        
        Args:
            dataset: Dataset root with all data
        """
        self.dataset = dataset
        self.stats_calculator = StatisticsCalculator()
    
    def get_experiments_for_comparison(
        self,
        marker_type_dir: str,
        experiment_name: str,
        sizes: List[str]
    ) -> Dict[str, Optional[Experiment]]:
        """
        Get experiments from different sizes for comparison.
        
        Args:
            marker_type_dir: Marker type directory name (e.g., "HD17_vert")
            experiment_name: Experiment name
            sizes: List of size codes (e.g., ["m40", "m80"])
            
        Returns:
            Dict mapping size code to Experiment (or None if not found)
        """
        experiments = {}
        
        for size_code in sizes:
            size_group = self.dataset.get_marker_size(size_code)
            if not size_group:
                experiments[size_code] = None
                continue
            
            marker_type = size_group.get_marker_type(marker_type_dir)
            if not marker_type:
                experiments[size_code] = None
                continue
            
            experiment = marker_type.get_experiment(experiment_name)
            
            # Calculate aggregated statistics if not already computed
            if experiment and not experiment.aggregated_stats:
                experiment.aggregated_stats = self.stats_calculator.calculate_experiment_statistics(experiment)
            
            experiments[size_code] = experiment
        
        return experiments
    
    def calculate_comparison_metrics(
        self,
        experiment: Experiment,
        size_code: str,
        size_display: str,
        total_markers: int = 70
    ) -> ComparisonMetrics:
        """
        Calculate comparison metrics for an experiment.
        
        Args:
            experiment: Experiment to analyze
            size_code: Size code (m40, etc.)
            size_display: Display name (40x40 мм)
            total_markers: Total number of possible markers
            
        Returns:
            ComparisonMetrics object
        """
        if not experiment or not experiment.aggregated_stats:
            return ComparisonMetrics(size_code, size_display)
        
        stats = experiment.aggregated_stats
        detections = stats.marker_detections
        
        # Качество детекции
        total_detections = sum(detections.values())
        detected_markers = len([v for v in detections.values() if v > 0])
        avg_per_marker = total_detections / total_markers if total_markers > 0 else 0
        coverage = (detected_markers / total_markers * 100) if total_markers > 0 else 0
        
        # Распределение
        detection_values = list(detections.values())
        std_dev = float(np.std(detection_values)) if detection_values else 0.0
        min_det = min(detection_values) if detection_values else 0
        max_det = max(detection_values) if detection_values else 0
        
        # FPS - соберем из всех attempts для расчета std_dev
        fps_values = []
        frames_with_det = 0
        total_frames = 0
        
        for attempt in experiment.attempts:
            if attempt.log_data:
                for frame in attempt.log_data.frames:
                    if len(frame.marker_ids) > 0:
                        frames_with_det += 1
                    total_frames += 1
        
        success_rate = (frames_with_det / total_frames * 100) if total_frames > 0 else 0
        fps_std = float(np.std([stats.fps_min, stats.fps_max, stats.fps_avg])) if stats.fps_avg > 0 else 0.0
        
        return ComparisonMetrics(
            size_code=size_code,
            size_display=size_display,
            total_detections=int(total_detections),
            avg_detections_per_marker=avg_per_marker,
            detection_coverage=coverage,
            detected_marker_count=detected_markers,
            std_dev=std_dev,
            min_detections=int(min_det),
            max_detections=int(max_det),
            fps_min=stats.fps_min,
            fps_max=stats.fps_max,
            fps_median=stats.fps_avg,
            fps_std_dev=fps_std,
            frames_with_detections=frames_with_det,
            total_frames=total_frames,
            detection_success_rate=success_rate,
            avg_simultaneous=stats.simultaneous_markers_avg,
            max_simultaneous=stats.simultaneous_markers_max,
            min_simultaneous=stats.simultaneous_markers_min
        )
    
    def compare_experiments(
        self,
        marker_type_dir: str,
        experiment_name: str,
        sizes: List[str],
        total_markers: int = 70
    ) -> Dict[str, ComparisonMetrics]:
        """
        Compare one experiment across multiple sizes.
        
        Args:
            marker_type_dir: Marker type directory name
            experiment_name: Experiment name
            sizes: List of size codes
            total_markers: Total number of markers
            
        Returns:
            Dict mapping size code to ComparisonMetrics
        """
        experiments = self.get_experiments_for_comparison(
            marker_type_dir,
            experiment_name,
            sizes
        )
        
        metrics = {}
        for size_code in sizes:
            experiment = experiments.get(size_code)
            size_group = self.dataset.get_marker_size(size_code)
            size_display = size_group.display_name if size_group else size_code
            
            metrics[size_code] = self.calculate_comparison_metrics(
                experiment,
                size_code,
                size_display,
                total_markers
            )
        
        return metrics
