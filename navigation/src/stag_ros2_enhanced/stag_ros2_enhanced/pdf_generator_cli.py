#!/usr/bin/env python3
"""
PDF Generator CLI

Command-line tool for generating printable PDF files with STag markers.
Places markers in a grid layout on A4 pages for easy printing and cutting.
"""

import argparse
import os
import sys
from pathlib import Path
from datetime import datetime
import yaml

from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from PIL import Image

# Optional ROS2 dependency - only used if available
try:
    from ament_index_python.packages import get_package_share_directory
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class MarkerPDFGenerator:
    """PDF generator for printable marker sheets"""
    
    def __init__(self, config_path=None):
        """
        Initialize PDF generator.
        
        Args:
            config_path: Path to YAML config file (optional)
        """
        self.config = self._load_config(config_path)
        self.markers = []
    
    def _load_config(self, config_path):
        """Load configuration from YAML file"""
        # Default configuration
        default_config = {
            'page_width_mm': 210,
            'page_height_mm': 297,
            'page_margin': 10,
            'marker_margin': 3,
            'alignment': 'top-left',  # or 'center'
            'show_cut_lines': True,
            'show_marker_id': True,
            'cut_line_color': [0.7, 0.7, 0.7],
            'cut_line_width': 0.5,
            'cut_line_dash': [2, 2],
            'font_name': 'Helvetica',
            'font_size': 6,
            'font_color': [0, 0, 0],
            'metadata': {
                'author': 'STag Marker Generator',
                'title': 'STag Markers for Detection',
                'subject': 'Fiducial markers for robotic localization',
                'creator': 'stag_ros2_enhanced PDF generator'
            }
        }
        
        # Load user configuration if provided
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = yaml.safe_load(f)
                    default_config.update(user_config)
                    print(f"✓ Loaded configuration from: {config_path}")
            except Exception as e:
                print(f"⚠ Warning: Could not load config file: {e}")
                print("  Using default configuration")
        else:
            if config_path:
                print(f"⚠ Warning: Config file not found: {config_path}")
            print("  Using default configuration")
        
        return default_config
    
    def load_markers(self, input_dir):
        """
        Load PNG markers from directory.
        
        Args:
            input_dir: Path to directory containing marker PNG files
            
        Returns:
            Number of loaded markers
        """
        input_path = Path(input_dir)
        
        if not input_path.exists():
            raise FileNotFoundError(f"Directory not found: {input_dir}")
        
        # Find all PNG files
        png_files = sorted(input_path.glob('*.png'))
        
        if not png_files:
            raise ValueError(f"No PNG files found in: {input_dir}")
        
        self.markers = []
        for png_file in png_files:
            try:
                # Verify file can be opened
                with Image.open(png_file) as img:
                    self.markers.append({
                        'path': str(png_file),
                        'name': png_file.stem,
                        'size': img.size
                    })
            except Exception as e:
                print(f"⚠ Warning: Could not load {png_file.name}: {e}")
        
        print(f"✓ Loaded {len(self.markers)} markers from {input_dir}")
        return len(self.markers)
    
    def calculate_layout(self, marker_size_mm):
        """
        Calculate marker layout on page.
        
        Args:
            marker_size_mm: Marker size in millimeters
            
        Returns:
            Dictionary with layout parameters
        """
        page_margin = self.config['page_margin']
        marker_margin = self.config['marker_margin']
        
        # Available area for markers
        usable_width = self.config['page_width_mm'] - 2 * page_margin
        usable_height = self.config['page_height_mm'] - 2 * page_margin
        
        # Marker ID takes additional space
        id_height = self.config['font_size'] * 0.5 if self.config['show_marker_id'] else 0
        marker_height_total = marker_size_mm + id_height
        
        # Calculate columns and rows
        cols = int((usable_width + marker_margin) / (marker_size_mm + marker_margin))
        rows = int((usable_height + marker_margin) / (marker_height_total + marker_margin))
        
        if cols < 1 or rows < 1:
            raise ValueError(
                f"Marker size {marker_size_mm}mm is too large for page! "
                f"Maximum size: {usable_width}×{usable_height}mm"
            )
        
        # Calculate alignment offset based on config
        alignment = self.config.get('alignment', 'top-left')
        
        if alignment == 'center':
            # Center on page
            actual_width = cols * marker_size_mm + (cols - 1) * marker_margin
            actual_height = rows * marker_height_total + (rows - 1) * marker_margin
            
            offset_x = page_margin + (usable_width - actual_width) / 2
            offset_y = page_margin + (usable_height - actual_height) / 2
        else:
            # Align to top-left (easier for cutting)
            offset_x = page_margin
            offset_y = self.config['page_height_mm'] - page_margin  # Start from top
        
        layout = {
            'cols': cols,
            'rows': rows,
            'per_page': cols * rows,
            'marker_size_mm': marker_size_mm,
            'marker_height_total': marker_height_total,
            'offset_x': offset_x,
            'offset_y': offset_y,
            'id_height': id_height,
            'alignment': alignment
        }
        
        print(f"✓ Layout: {cols}×{rows} markers per page ({layout['per_page']} total)")
        print(f"✓ Alignment: {alignment}")
        return layout
    
    def draw_cut_lines(self, c, x, y, width, height):
        """Draw dashed cut lines around marker"""
        if not self.config['show_cut_lines']:
            return
        
        c.saveState()
        color = self.config['cut_line_color']
        c.setStrokeColorRGB(*color)
        c.setLineWidth(self.config['cut_line_width'])
        c.setDash(*self.config['cut_line_dash'])
        
        # Draw rectangle around marker
        c.rect(x, y, width, height, stroke=1, fill=0)
        
        c.restoreState()
    
    def draw_marker_id(self, c, x, y, marker_name, width):
        """Draw marker ID text below marker"""
        if not self.config['show_marker_id']:
            return
        
        c.saveState()
        c.setFont(self.config['font_name'], self.config['font_size'])
        color = self.config['font_color']
        c.setFillColorRGB(*color)
        
        # Center text below marker
        text_width = c.stringWidth(marker_name, self.config['font_name'], self.config['font_size'])
        text_x = x + (width - text_width) / 2
        text_y = y - self.config['font_size'] * 1.2
        
        c.drawString(text_x, text_y, marker_name)
        c.restoreState()
    
    def generate_pdf(self, marker_size_mm, output_path):
        """
        Generate PDF file with markers.
        
        Args:
            marker_size_mm: Marker size in millimeters
            output_path: Path to save PDF file
        """
        if not self.markers:
            raise ValueError("No markers loaded. Call load_markers() first.")
        
        # Calculate layout
        layout = self.calculate_layout(marker_size_mm)
        
        # Create canvas
        c = canvas.Canvas(output_path, pagesize=A4)
        
        # Set metadata
        metadata = self.config['metadata']
        c.setAuthor(metadata['author'])
        c.setTitle(metadata['title'])
        c.setSubject(metadata['subject'])
        c.setCreator(metadata['creator'])
        c.setProducer(f"{metadata['creator']} - {datetime.now().strftime('%Y-%m-%d')}")
        
        marker_size_pt = marker_size_mm * mm
        marker_margin_pt = self.config['marker_margin'] * mm
        id_height_pt = layout['id_height'] * mm
        
        total_pages = (len(self.markers) + layout['per_page'] - 1) // layout['per_page']
        
        print(f"✓ Generating PDF: {len(self.markers)} markers on {total_pages} pages...")
        
        marker_idx = 0
        for page_num in range(total_pages):
            if page_num > 0:
                c.showPage()
            
            markers_on_page = 0
            
            for row in range(layout['rows']):
                for col in range(layout['cols']):
                    if marker_idx >= len(self.markers):
                        break
                    
                    marker = self.markers[marker_idx]
                    
                    # Calculate position (X: left to right)
                    x = (layout['offset_x'] + col * (marker_size_mm + self.config['marker_margin'])) * mm
                    
                    # Calculate Y position based on alignment
                    if layout['alignment'] == 'center':
                        # Y from bottom (standard ReportLab)
                        y = (layout['offset_y'] + row * (layout['marker_height_total'] + self.config['marker_margin'])) * mm
                    else:
                        # Y from top (top-left alignment)
                        y_from_top = layout['offset_y'] - row * (layout['marker_height_total'] + self.config['marker_margin']) - marker_size_mm
                        y = y_from_top * mm
                    
                    # Draw marker image
                    c.drawImage(
                        marker['path'],
                        x, y,
                        width=marker_size_pt,
                        height=marker_size_pt,
                        preserveAspectRatio=True
                    )
                    
                    # Draw cut lines
                    self.draw_cut_lines(c, x, y, marker_size_pt, marker_size_pt)
                    
                    # Draw marker ID
                    self.draw_marker_id(c, x, y, marker['name'], marker_size_pt)
                    
                    marker_idx += 1
                    markers_on_page += 1
                
                if marker_idx >= len(self.markers):
                    break
            
            print(f"  Page {page_num + 1}/{total_pages}: {markers_on_page} markers")
        
        c.save()
        print(f"✓ PDF saved: {output_path}")
        
        # File info
        file_size = os.path.getsize(output_path)
        print(f"  File size: {file_size / 1024:.1f} KB")


def main():
    """Main entry point for PDF generator"""
    
    parser = argparse.ArgumentParser(
        description='Generate printable PDF files with STag markers',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate PDF with default settings
  ros2 run stag_ros2_enhanced generate_pdf.py --input markers/HG17 --marker-size 50
  
  # Generate with custom output filename
  ros2 run stag_ros2_enhanced generate_pdf.py --input markers/HD13 --marker-size 80 --output custom_name.pdf
  
  # Generate with custom config
  ros2 run stag_ros2_enhanced generate_pdf.py --input markers/HG17 --marker-size 50 --config my_config.yaml
        """
    )
    
    parser.add_argument(
        '--input',
        required=True,
        help='Path to directory containing PNG marker images'
    )
    
    parser.add_argument(
        '--marker-size',
        type=float,
        required=True,
        help='Marker size in millimeters'
    )
    
    parser.add_argument(
        '--output',
        help='Path to save PDF file (optional, default: markers_[folder]_[size]mm.pdf)'
    )
    
    parser.add_argument(
        '--config',
        help='Path to YAML configuration file (optional)'
    )
    
    args = parser.parse_args()
    
    # Determine output filename
    if args.output:
        output_path = args.output
    else:
        folder_name = Path(args.input).name
        output_path = f"markers_{folder_name}_{int(args.marker_size)}mm.pdf"
    
    try:
        # Create generator
        generator = MarkerPDFGenerator(config_path=args.config)
        
        # Load markers
        generator.load_markers(args.input)
        
        # Generate PDF
        generator.generate_pdf(args.marker_size, output_path)
        
        print("\n✅ Done! PDF file successfully created.")
        
    except Exception as e:
        print(f"\n❌ Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
