"""Statistics calculator for marker detection experiments."""

from typing import List, Dict
from collections import defaultdict
import numpy as np
from ..models.log_data import LogData
from ..models.experiment import Experiment
from ..models.statistics import Statistics


class StatisticsCalculator:
    """Calculator for experiment statistics."""
    
    @staticmethod
    def calculate_from_log(log_data: LogData) -> Statistics:
        """
        Calculate statistics from a single log file.
        
        Args:
            log_data: Parsed log data
            
        Returns:
            Statistics object
        """
        if not log_data or not log_data.frames:
            return Statistics()
        
        # Count marker detections
        marker_detections = defaultdict(int)
        
        # Track FPS and simultaneous markers
        fps_values = []
        simultaneous_counts = []
        
        frames = log_data.frames
        
        for i, frame in enumerate(frames):
            # Count detections for each marker
            for marker_id in frame.marker_ids:
                marker_detections[marker_id] += 1
            
            # Track how many markers were seen simultaneously
            simultaneous_counts.append(len(frame.marker_ids))
            
            # Calculate FPS from timestamp differences
            if i > 0:
                time_diff = frame.timestamp - frames[i-1].timestamp
                if time_diff > 0:  # Avoid division by zero
                    fps = 1.0 / time_diff
                    fps_values.append(fps)
        
        # Calculate FPS statistics (using median instead of average)
        fps_min = min(fps_values) if fps_values else 0.0
        fps_max = max(fps_values) if fps_values else 0.0
        fps_avg = float(np.median(fps_values)) if fps_values else 0.0  # Using median
        
        # Calculate simultaneous markers statistics
        sim_min = min(simultaneous_counts) if simultaneous_counts else 0
        sim_max = max(simultaneous_counts) if simultaneous_counts else 0
        sim_avg = sum(simultaneous_counts) / len(simultaneous_counts) if simultaneous_counts else 0.0
        
        return Statistics(
            marker_detections=dict(marker_detections),
            fps_min=fps_min,
            fps_max=fps_max,
            fps_avg=fps_avg,
            simultaneous_markers_min=sim_min,
            simultaneous_markers_max=sim_max,
            simultaneous_markers_avg=sim_avg,
            total_frames=len(frames)
        )
    
    @staticmethod
    def aggregate_statistics(stats_list: List[Statistics]) -> Statistics:
        """
        Aggregate statistics from multiple attempts.
        
        Marker detections are averaged across attempts.
        FPS: min/max across attempts, median of per-attempt median FPS.
        Simultaneous markers: min/max across attempts, mean of per-attempt averages.
        
        Args:
            stats_list: List of Statistics from different attempts
            
        Returns:
            Aggregated Statistics
        """
        if not stats_list:
            return Statistics()
        
        # Aggregate marker detections (average across attempts)
        all_marker_ids = set()
        for stats in stats_list:
            all_marker_ids.update(stats.marker_detections.keys())
        
        aggregated_detections = {}
        for marker_id in all_marker_ids:
            # Average detections across attempts
            total = sum(stats.marker_detections.get(marker_id, 0) for stats in stats_list)
            aggregated_detections[marker_id] = total / len(stats_list)
        
        # Aggregate FPS (using median)
        all_fps_min = [s.fps_min for s in stats_list if s.fps_min > 0]
        all_fps_max = [s.fps_max for s in stats_list if s.fps_max > 0]
        all_fps_avg = [s.fps_avg for s in stats_list if s.fps_avg > 0]
        
        fps_min = min(all_fps_min) if all_fps_min else 0.0
        fps_max = max(all_fps_max) if all_fps_max else 0.0
        fps_avg = float(np.median(all_fps_avg)) if all_fps_avg else 0.0  # Median of medians
        
        # Aggregate simultaneous markers
        all_sim_min = [s.simultaneous_markers_min for s in stats_list]
        all_sim_max = [s.simultaneous_markers_max for s in stats_list]
        all_sim_avg = [s.simultaneous_markers_avg for s in stats_list if s.simultaneous_markers_avg > 0]
        
        sim_min = min(all_sim_min) if all_sim_min else 0
        sim_max = max(all_sim_max) if all_sim_max else 0
        sim_avg = sum(all_sim_avg) / len(all_sim_avg) if all_sim_avg else 0.0
        
        # Average total frames
        avg_frames = sum(s.total_frames for s in stats_list) / len(stats_list) if stats_list else 0
        
        return Statistics(
            marker_detections=aggregated_detections,
            fps_min=fps_min,
            fps_max=fps_max,
            fps_avg=fps_avg,
            simultaneous_markers_min=sim_min,
            simultaneous_markers_max=sim_max,
            simultaneous_markers_avg=sim_avg,
            total_frames=int(avg_frames)
        )
    
    @staticmethod
    def calculate_experiment_statistics(experiment: Experiment) -> Statistics:
        """
        Calculate aggregated statistics for an entire experiment.
        
        Args:
            experiment: Experiment with multiple attempts
            
        Returns:
            Aggregated Statistics
        """
        stats_list = []
        
        for attempt in experiment.attempts:
            if attempt.log_data:
                stats = StatisticsCalculator.calculate_from_log(attempt.log_data)
                stats_list.append(stats)
        
        if not stats_list:
            return Statistics()
        
        aggregated = StatisticsCalculator.aggregate_statistics(stats_list)
        experiment.aggregated_stats = aggregated
        
        return aggregated
