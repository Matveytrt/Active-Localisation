"""Comparison visualizer for side-by-side heatmaps and metrics."""

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pathlib import Path
from typing import Dict, List

from stag_data_analyzer.models.experiment import Experiment
from stag_data_analyzer.models.comparison_metrics import ComparisonMetrics


class ComparisonVisualizer:
    """Visualizer for comparison results."""
    
    def __init__(self, config: Dict):
        """
        Initialize visualizer.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.marker_config = config.get('markers', {})
        self.viz_config = config.get('visualization', {}).get('heatmap', {})
    
    def generate_side_by_side_heatmaps(
        self,
        experiments: Dict[str, Experiment],
        marker_type: str,
        experiment_name: str,
        output_path: Path,
        sizes_order: List[str],
        output_formats: List[str] = None
    ) -> None:
        """
        Generate side-by-side heatmaps for comparison.
        
        Args:
            experiments: Dict mapping size code to Experiment
            marker_type: Marker type name
            experiment_name: Experiment name
            output_path: Output file path (without extension)
            sizes_order: Order of sizes for display
            output_formats: List of formats to save ['html', 'png', 'csv']
        """
        if output_formats is None:
            output_formats = ['html', 'png']
        rows = self.marker_config.get('rows', 7)
        cols = self.marker_config.get('cols', 10)
        
        # Create subplots
        n_sizes = len(sizes_order)
        fig = make_subplots(
            rows=1,
            cols=n_sizes,
            subplot_titles=[self._get_size_display(size) 
                          if experiments.get(size) and experiments[size].aggregated_stats
                          else f"{self._get_size_display(size)}<br>No data"
                          for size in sizes_order],
            horizontal_spacing=0.05
        )
        
        # Add heatmaps
        for idx, size_code in enumerate(sizes_order, 1):
            experiment = experiments.get(size_code)
            
            if experiment and experiment.aggregated_stats:
                heatmap_data = self._create_heatmap_matrix(
                    experiment.aggregated_stats.marker_detections,
                    rows,
                    cols
                )
                text_data = self._create_text_matrix(heatmap_data, rows, cols, flipped=False)
            else:
                heatmap_data = np.zeros((rows, cols))
                text_data = np.full((rows, cols), "No data", dtype=object)
            
            # Add heatmap trace
            fig.add_trace(
                go.Heatmap(
                    z=heatmap_data,
                    text=text_data,
                    texttemplate="%{text}",
                    textfont={"size": self.viz_config.get('cell_font_size', 10)},
                    colorscale=self.viz_config.get('colorscale', 'Viridis'),
                    showscale=(idx == n_sizes),  # Show scale only for last plot
                    colorbar=dict(
                        title="Detections",
                        x=1.02
                    ) if idx == n_sizes else None
                ),
                row=1,
                col=idx
            )
        
        # Update layout
        title = (
            f"<b>Size Comparison: {marker_type}</b><br>"
            f"Experiment: {experiment_name}"
        )
        
        fig.update_layout(
            title={
                'text': title,
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            height=800,
            width=500 * n_sizes,
            showlegend=False
        )
        
        # Update axes
        # Note: go.Heatmap displays matrix[0,:] at bottom by default (mathematical coordinate system)
        # This means ID 0 (first row) is already at the bottom - no need to reverse!
        for i in range(1, n_sizes + 1):
            fig.update_xaxes(title_text="Column", row=1, col=i, dtick=1)
            fig.update_yaxes(
                title_text="Row" if i == 1 else "", 
                row=1, 
                col=i, 
                dtick=1
            )
        
        # Save in requested formats
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        if 'html' in output_formats:
            fig.write_html(str(output_path.with_suffix('.html')))
            print(f"✓ Saved HTML: {output_path.with_suffix('.html').name}")
        
        if 'png' in output_formats:
            try:
                fig.write_image(str(output_path.with_suffix('.png')), scale=2)
                print(f"✓ Saved PNG: {output_path.with_suffix('.png').name}")
            except Exception as e:
                print(f"⚠️  Warning: Could not save PNG: {e}")
    
    def generate_metrics_table(
        self,
        metrics: Dict[str, ComparisonMetrics],
        marker_type: str,
        experiment_name: str,
        output_path: Path,
        sizes_order: List[str],
        output_formats: List[str] = None
    ) -> None:
        """
        Generate comparison metrics table.
        
        Args:
            metrics: Dict mapping size code to ComparisonMetrics
            marker_type: Marker type name
            experiment_name: Experiment name
            output_path: Output file path (without extension)
            sizes_order: Order of sizes
            output_formats: List of formats to save ['html', 'png', 'csv']
        """
        if output_formats is None:
            output_formats = ['html', 'csv']
        
        # Prepare table data
        metric_names = [
            "Total Detections",
            "Avg per Marker",
            "Coverage (%)",
            "Detected Markers",
            "Std Dev",
            "FPS (median)",
            "FPS Std Dev",
            "Success Rate (%)",
            "Avg Simultaneous",
            "Max Simultaneous"
        ]
        
        header = ["Metric"] + [metrics[size].size_display for size in sizes_order if size in metrics]
        
        table_data = []
        for metric_name in metric_names:
            row = [metric_name]
            for size in sizes_order:
                if size not in metrics:
                    row.append("N/A")
                    continue
                
                m = metrics[size]
                if metric_name == "Total Detections":
                    row.append(f"{m.total_detections}")
                elif metric_name == "Avg per Marker":
                    row.append(f"{m.avg_detections_per_marker:.1f}")
                elif metric_name == "Coverage (%)":
                    row.append(f"{m.detection_coverage:.1f}%")
                elif metric_name == "Detected Markers":
                    row.append(f"{m.detected_marker_count}/70")
                elif metric_name == "Std Dev":
                    row.append(f"{m.std_dev:.2f}")
                elif metric_name == "FPS (median)":
                    row.append(f"{m.fps_median:.1f}")
                elif metric_name == "FPS Std Dev":
                    row.append(f"{m.fps_std_dev:.2f}")
                elif metric_name == "Success Rate (%)":
                    row.append(f"{m.detection_success_rate:.1f}%")
                elif metric_name == "Avg Simultaneous":
                    row.append(f"{m.avg_simultaneous:.1f}")
                elif metric_name == "Max Simultaneous":
                    row.append(f"{m.max_simultaneous}")
            
            table_data.append(row)
        
        # Create figure
        fig = go.Figure(data=[go.Table(
            header=dict(
                values=header,
                fill_color='paleturquoise',
                align='left',
                font=dict(size=14, color='black')
            ),
            cells=dict(
                values=list(zip(*([row] for row in table_data))),
                fill_color='lavender',
                align='left',
                font=dict(size=12)
            )
        )])
        
        fig.update_layout(
            title=f"<b>Metrics Comparison: {marker_type}</b><br>Experiment: {experiment_name}",
            width=300 + len(sizes_order) * 200,
            height=600
        )
        
        # Save in requested formats
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        if 'html' in output_formats:
            fig.write_html(str(output_path.with_suffix('.html')))
            print(f"✓ Saved HTML: {output_path.with_suffix('.html').name}")
        
        if 'csv' in output_formats:
            csv_path = output_path.with_suffix('.csv')
            with open(csv_path, 'w') as f:
                f.write(','.join(header) + '\n')
                for row in table_data:
                    f.write(','.join(str(v) for v in row) + '\n')
            print(f"✓ Saved CSV: {csv_path.name}")
    
    def _get_size_display(self, size_code: str) -> str:
        """Get display name for size code."""
        size_map = {cfg['name']: cfg['display'] 
                    for cfg in self.config.get('marker_sizes', [])}
        return size_map.get(size_code, size_code)
    
    def _create_heatmap_matrix(
        self,
        marker_detections: Dict[int, float],
        rows: int,
        cols: int
    ) -> np.ndarray:
        """Create heatmap matrix from detections."""
        matrix = np.zeros((rows, cols))
        for marker_id, count in marker_detections.items():
            row = marker_id // cols
            col = marker_id % cols
            if row < rows and col < cols:
                matrix[row, col] = count
        return matrix
    
    def _create_text_matrix(
        self,
        heatmap_data: np.ndarray,
        rows: int,
        cols: int,
        flipped: bool = False
    ) -> np.ndarray:
        """Create text labels for cells."""
        text_matrix = np.empty((rows, cols), dtype=object)
        for row in range(rows):
            for col in range(cols):
                if flipped:
                    original_row = rows - 1 - row
                    marker_id = original_row * cols + col
                else:
                    marker_id = row * cols + col
                count = heatmap_data[row, col]
                text_matrix[row, col] = f"ID:{marker_id}<br>{count:.0f}"
        return text_matrix
