"""Heatmap generator using Plotly for marker detection visualization."""

import numpy as np
import plotly.express as px
from pathlib import Path
from typing import Dict, Optional
from ..models.statistics import Statistics


class HeatmapGenerator:
    """Generator for marker detection heatmaps."""
    
    def __init__(self, config: Dict):
        """
        Initialize heatmap generator with configuration.
        
        Args:
            config: Configuration dictionary from YAML
        """
        self.config = config
        self.viz_config = config.get('visualization', {}).get('heatmap', {})
        self.marker_config = config.get('markers', {})
    
    def generate_heatmap(
        self,
        statistics: Statistics,
        marker_size: str,
        marker_type: str,
        experiment_name: str,
        output_path: Path
    ) -> None:
        """
        Generate and save a heatmap for marker detections.
        
        Args:
            statistics: Aggregated statistics with marker detections
            marker_size: Marker size display name (e.g., "40x40 мм")
            marker_type: Marker type name (e.g., "HD17 (vert)")
            experiment_name: Experiment name
            output_path: Path where to save the PNG file
        """
        # Get heatmap dimensions from config
        rows = self.marker_config.get('rows', 7)
        cols = self.marker_config.get('cols', 10)
        
        # Create heatmap matrix
        heatmap_data = self._create_heatmap_matrix(
            statistics.marker_detections,
            rows,
            cols
        )
        
        # Flip matrix vertically so ID 0 is at bottom and ID 69 at top
        heatmap_data = np.flipud(heatmap_data)
        
        # Create labels for display (after flipping)
        text_data = self._create_text_matrix(heatmap_data, rows, cols, flipped=True)
        
        # Create title with experiment information
        title = self._create_title(
            marker_size,
            marker_type,
            experiment_name,
            statistics
        )
        
        # Generate figure using Plotly
        fig = self._create_plotly_figure(
            heatmap_data,
            text_data,
            title,
            statistics  # Pass statistics for annotation
        )
        
        # Save to file
        self._save_figure(fig, output_path)
        
        print(f"✓ Heatmap saved: {output_path}")
    
    def _create_heatmap_matrix(
        self,
        marker_detections: Dict[int, float],
        rows: int,
        cols: int
    ) -> np.ndarray:
        """
        Create a 2D matrix for heatmap from marker detections.
        
        Args:
            marker_detections: Dict mapping marker_id to detection count
            rows: Number of rows
            cols: Number of columns
            
        Returns:
            2D numpy array with detection counts
        """
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
        """
        Create text labels for each cell showing marker ID and count.
        
        Args:
            heatmap_data: Detection count matrix (already flipped if needed)
            rows: Number of rows
            cols: Number of columns
            flipped: Whether the matrix was flipped vertically
            
        Returns:
            2D array of text labels
        """
        text_matrix = np.empty((rows, cols), dtype=object)
        
        for row in range(rows):
            for col in range(cols):
                if flipped:
                    # Calculate original marker ID (before flip)
                    # After flip: top row (row=0) corresponds to bottom original row
                    original_row = rows - 1 - row
                    marker_id = original_row * cols + col
                else:
                    marker_id = row * cols + col
                    
                count = heatmap_data[row, col]
                text_matrix[row, col] = f"ID:{marker_id}<br>{count:.0f}"
        
        return text_matrix
    
    def _create_title(
        self,
        marker_size: str,
        marker_type: str,
        experiment_name: str,
        statistics: Statistics
    ) -> str:
        """
        Create a title with main experiment information.
        Statistics will be shown as annotations instead.
        
        Args:
            marker_size: Marker size display name
            marker_type: Marker type name
            experiment_name: Experiment name
            statistics: Statistics with FPS and visibility data
            
        Returns:
            Formatted title string
        """
        # Only main info in title
        title = (
            f"<b>Marker Detection Heatmap</b><br>"
            f"Size: {marker_size} | Type: {marker_type} | Experiment: {experiment_name}"
        )
        
        return title
    
    def _create_stats_annotation(self, statistics: Statistics) -> str:
        """
        Create statistics text for annotation.
        
        Args:
            statistics: Statistics object
            
        Returns:
            Formatted statistics string
        """
        return (
            f"<b>Statistics:</b><br>"
            f"FPS: Min={statistics.fps_min:.1f}, Max={statistics.fps_max:.1f}, Median={statistics.fps_avg:.1f}<br>"
            f"Simultaneous Markers: Min={statistics.simultaneous_markers_min}, "
            f"Max={statistics.simultaneous_markers_max}, Avg={statistics.simultaneous_markers_avg:.1f}<br>"
            f"Total Frames: {statistics.total_frames}"
        )
    
    def _create_plotly_figure(
        self,
        heatmap_data: np.ndarray,
        text_data: np.ndarray,
        title: str,
        statistics: Statistics
    ):
        """
        Create Plotly figure with heatmap and annotations.
        
        Args:
            heatmap_data: Detection count matrix
            text_data: Text labels matrix
            title: Figure title
            statistics: Statistics for annotation box
            
        Returns:
            Plotly figure object
        """
        # Get visualization settings
        colorscale = self.viz_config.get('colorscale', 'Viridis')
        show_values = self.viz_config.get('show_values', True)
        title_font_size = self.viz_config.get('title_font_size', 20)
        cell_font_size = self.viz_config.get('cell_font_size', 10)
        width = self.viz_config.get('width', 1400)
        height = self.viz_config.get('height', 900)
        
        # Create figure using plotly.express
        fig = px.imshow(
            heatmap_data,
            color_continuous_scale=colorscale,
            aspect="auto",
            labels=dict(color="Detections"),
        )
        
        # Add text annotations if enabled
        if show_values:
            fig.update_traces(
                text=text_data,
                texttemplate="%{text}",
                textfont={"size": cell_font_size}
            )
        
        # Update layout
        fig.update_layout(
            title={
                'text': title,
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': title_font_size}
            },
            width=width,
            height=height,
            xaxis_title="Column",
            yaxis_title="Row",
            coloraxis_colorbar=dict(
                title="Detection<br>Count",
                thicknessmode="pixels",
                thickness=20,
                lenmode="pixels",
                len=300,
            )
        )
        
        # Add statistics annotation in top-right corner
        stats_text = self._create_stats_annotation(statistics)
        fig.add_annotation(
            text=stats_text,
            xref="paper",
            yref="paper",
            x=0.98,  # Right side
            y=0.98,  # Top
            xanchor="right",
            yanchor="top",
            showarrow=False,
            bgcolor="rgba(255, 255, 255, 0.9)",
            bordercolor="black",
            borderwidth=1,
            borderpad=10,
            font=dict(size=12, family="monospace")
        )
        
        # Update axes
        fig.update_xaxes(side="bottom", dtick=1)
        fig.update_yaxes(dtick=1)
        
        return fig
    
    def _save_figure(self, fig, output_path: Path) -> None:
        """
        Save Plotly figure to file.
        
        Args:
            fig: Plotly figure object
            output_path: Path where to save the file
        """
        # Create output directory if it doesn't exist
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Get output settings
        output_format = self.config.get('visualization', {}).get('output_format', 'png')
        dpi = self.config.get('visualization', {}).get('dpi', 300)
        
        # Calculate scale factor for DPI
        # Plotly's default is 72 DPI, so we scale accordingly
        scale = dpi / 72.0
        
        # Save figure
        if output_format == 'png':
            fig.write_image(
                str(output_path),
                format='png',
                scale=scale
            )
        elif output_format == 'html':
            fig.write_html(str(output_path))
        else:
            # Default to PNG
            fig.write_image(
                str(output_path),
                format='png',
                scale=scale
            )
