#!/usr/bin/env python3
"""
Batch PDF Generator CLI

Automatically generate PDF files for multiple marker folders and sizes.
Wrapped by scripts/pdf_batch_generation.py and scripts/pdf_batch_generation.sh.
"""

import argparse
import os
import sys
import time
from pathlib import Path

from stag_ros2_enhanced.pdf_generator_cli import MarkerPDFGenerator


# ANSI color codes
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'  # No Color


def print_header(text):
    """Print header with blue color"""
    print(f"{Colors.BLUE}{'=' * 60}{Colors.NC}")
    print(f"{Colors.BLUE}{text}{Colors.NC}")
    print(f"{Colors.BLUE}{'=' * 60}{Colors.NC}")


def print_success(text):
    """Print success message with green checkmark"""
    print(f"{Colors.GREEN}✓{Colors.NC} {text}")


def print_error(text):
    """Print error message with red X"""
    print(f"{Colors.RED}✗{Colors.NC} {text}")


def print_warning(text):
    """Print warning message with yellow triangle"""
    print(f"{Colors.YELLOW}⚠{Colors.NC} {text}")


def print_info(text):
    """Print info message with blue i"""
    print(f"{Colors.BLUE}ℹ{Colors.NC} {text}")


def generate_single_pdf(folder, size, output_dir, config_file):
    """
    Generate a single PDF file.
    
    Args:
        folder: Input marker folder path
        size: Marker size in mm
        output_dir: Output directory for PDFs
        config_file: Path to config YAML
        
    Returns:
        True if successful, False otherwise
    """
    folder_name = Path(folder).name
    output_file = f"markers_{folder_name}_{int(size)}mm.pdf"
    
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, output_file)
    
    print()
    print_info(f"Folder: {folder} | Size: {size}mm")
    print_info(f"Output file: {output_file}")
    
    try:
        # Create generator
        generator = MarkerPDFGenerator(config_path=config_file)
        
        # Load markers
        generator.load_markers(folder)
        
        # Generate PDF
        generator.generate_pdf(size, output_file)
        
        # Check file was created
        if os.path.isfile(output_file):
            file_size = os.path.getsize(output_file) / 1024  # KB
            print_success(f"Created: {output_file} ({file_size:.1f} KB)")
            return True
        else:
            print_error("File was not created")
            return False
            
    except Exception as e:
        print_error(f"Error during PDF generation: {e}")
        return False


def main():
    """Main entry point for batch PDF generator"""
    
    parser = argparse.ArgumentParser(
        description='Batch generate PDF files for multiple marker folders and sizes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use built-in default folders and sizes
  ros2 run stag_ros2_enhanced pdf_batch_generation.py
  
  # Specify custom folders and sizes
  ros2 run stag_ros2_enhanced pdf_batch_generation.py --folders markers/HD13 markers/HD17 --sizes 40 80 100
  
  # With custom output directory and config
  ros2 run stag_ros2_enhanced pdf_batch_generation.py --folders markers/HD17 --sizes 50 --output-dir output/pdfs --config my_config.yaml
        """
    )
    
    parser.add_argument(
        '--folders',
        nargs='+',
        help='List of marker folders to process (if not specified, uses default list)'
    )
    
    parser.add_argument(
        '--sizes',
        nargs='+',
        type=float,
        help='List of marker sizes in mm (if not specified, uses default list)'
    )
    
    parser.add_argument(
        '--output-dir',
        default='',
        help='Output directory for PDF files (optional, default: current directory)'
    )
    
    parser.add_argument(
        '--config',
        help='Path to YAML configuration file (optional)'
    )
    
    args = parser.parse_args()
    
    # Default configuration
    if args.folders:
        folders = args.folders
    else:
        # Default folders
        folders = [
            "HD13",
            "HD15",
            "HD17",
            "HO17",
            "HC21",
            "HG17/HG17_076-178",
            "HG17/HG17_117-137",
            "HG17/HG17_178-076",
            "HG17/HG17_255-000"
        ]
    
    if args.sizes:
        sizes = args.sizes
    else:
        # Default sizes
        sizes = [40, 80, 100]
    
    # Start batch generation
    print_header("Batch PDF Generator for STag Markers")
    
    # Validate folders exist
    valid_folders = []
    for folder in folders:
        if os.path.isdir(folder):
            valid_folders.append(folder)
        else:
            print_warning(f"Folder not found, skipping: {folder}")
    
    if not valid_folders:
        print_error("No valid folders found!")
        sys.exit(1)
    
    # Calculate total tasks
    total_tasks = len(valid_folders) * len(sizes)
    current_task = 0
    successful = 0
    failed = 0
    
    print()
    print_header("Starting Generation")
    print_info(f"Folders: {len(valid_folders)}")
    print_info(f"Sizes: {len(sizes)}")
    print_info(f"Total tasks: {total_tasks}")
    
    # Start timer
    start_time = time.time()
    
    # Generate PDFs for all combinations
    for folder in valid_folders:
        for size in sizes:
            current_task += 1
            
            print()
            print_header(f"Task {current_task}/{total_tasks}")
            
            if generate_single_pdf(folder, size, args.output_dir, args.config):
                successful += 1
            else:
                failed += 1
    
    # End timer
    end_time = time.time()
    duration = int(end_time - start_time)
    
    # Summary statistics
    print()
    print_header("Results")
    print_success(f"Successful: {successful}")
    
    if failed > 0:
        print_error(f"Failed: {failed}")
    else:
        print_info(f"Failed: {failed}")
    
    print_info(f"Execution time: {duration}s")
    
    # List created files
    print()
    print_header("Created PDF Files")
    
    if args.output_dir:
        pdf_dir = args.output_dir
    else:
        pdf_dir = "."
    
    try:
        pdf_files = list(Path(pdf_dir).glob("markers_*.pdf"))
        if pdf_files:
            for pdf_file in sorted(pdf_files):
                size = pdf_file.stat().st_size / 1024  # KB
                print(f"  {pdf_file.name} ({size:.1f} KB)")
        else:
            print_warning("No PDF files found")
    except Exception as e:
        print_warning(f"Could not list files: {e}")
    
    # Final message
    print()
    if failed == 0:
        print_success("All tasks completed successfully! 🎉")
        sys.exit(0)
    else:
        print_warning("Completed with errors")
        sys.exit(1)


if __name__ == "__main__":
    main()
