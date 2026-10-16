"""Comparison metrics data model."""

from dataclasses import dataclass
from typing import Dict


@dataclass
class ComparisonMetrics:
    """
    Метрики для сравнения маркеров.
    
    Attributes:
        size_code: Код размера (m40, m80, etc.)
        size_display: Отображаемое название (40x40 мм)
        
        # Качество детекции
        total_detections: int
        avg_detections_per_marker: float
        detection_coverage: float  # % маркеров обнаруженных хотя бы раз
        detected_marker_count: int  # Количество уникальных обнаруженных маркеров
        
        # Распределение детекций
        std_dev: float
        min_detections: int
        max_detections: int
        
        # Производительность
        fps_min: float
        fps_max: float
        fps_median: float
        fps_std_dev: float
        
        # Успешность
        frames_with_detections: int
        total_frames: int
        detection_success_rate: float
        
        # Одновременная видимость
        avg_simultaneous: float
        max_simultaneous: int
        min_simultaneous: int
    """
    
    size_code: str
    size_display: str
    
    # Качество детекции
    total_detections: int = 0
    avg_detections_per_marker: float = 0.0
    detection_coverage: float = 0.0
    detected_marker_count: int = 0
    
    # Распределение
    std_dev: float = 0.0
    min_detections: int = 0
    max_detections: int = 0
    
    # Производительность
    fps_min: float = 0.0
    fps_max: float = 0.0
    fps_median: float = 0.0
    fps_std_dev: float = 0.0
    
    # Успешность
    frames_with_detections: int = 0
    total_frames: int = 0
    detection_success_rate: float = 0.0
    
    # Одновременная видимость
    avg_simultaneous: float = 0.0
    max_simultaneous: int = 0
    min_simultaneous: int = 0
