#!/usr/bin/env python3
"""
Marker Generator CLI

Command-line tool for generating STag markers (HC, HG, HO types)
from standard HD marker sets.
"""

import argparse
import os
import sys

from stag_ros2_enhanced.marker_generators.high_capacity import generate_high_capacity_markers
from stag_ros2_enhanced.marker_generators.hue_greyscale import generate_hue_greyscale_markers
from stag_ros2_enhanced.marker_generators.high_occlusion import generate_high_occlusion_markers


def main():
    """Main entry point for marker generator"""
    
    # Create argument parser
    parser = argparse.ArgumentParser(
        description="Generate STag markers (HC, HG, HO types) from HD marker sets.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate high-capacity markers
  ros2 run stag_ros2_enhanced generate_markers.py --input-dir markers/HD17 --output-dir output/HC17 --input-set HD17 --output-set HC17
  
  # Generate hue-greyscale markers
  ros2 run stag_ros2_enhanced generate_markers.py --input-dir markers/HD17 --output-dir output/HG17 --input-set HD17 --output-set HG17
  
  # Generate high-occlusion markers with limited sample
  ros2 run stag_ros2_enhanced generate_markers.py --input-dir markers/HD17 --output-dir output/HO17 --input-set HD17 --output-set HO17 --sample-size 100
        """
    )
    
    # Add arguments
    parser.add_argument(
        '--input-dir',
        type=str,
        required=True,
        help='Path to input directory containing HD marker images'
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        required=True,
        help='Path to output directory for generated markers'
    )
    
    parser.add_argument(
        '--input-set',
        type=str,
        required=True,
        help='Name of input marker set (e.g., HD17, HD21, HD23)'
    )
    
    parser.add_argument(
        '--output-set',
        type=str,
        required=True,
        help='Name of output marker set (must start with HC, HG, or HO)'
    )
    
    parser.add_argument(
        '--sample-size',
        type=int,
        required=False,
        default=-1,
        help='Number of markers to generate (HO only, -1 for all)'
    )
    
    # Parse arguments
    args = parser.parse_args()
    
    # Validate input directory
    if not os.path.isdir(args.input_dir):
        print(f"Error: Input directory '{args.input_dir}' does not exist or is not a directory.", file=sys.stderr)
        sys.exit(1)
    
    # Create output directory if needed
    if not os.path.exists(args.output_dir):
        try:
            os.makedirs(args.output_dir)
            print(f"Created output directory: {args.output_dir}")
        except Exception as e:
            print(f"Error: Could not create output directory '{args.output_dir}'. {e}", file=sys.stderr)
            sys.exit(1)
    elif not os.path.isdir(args.output_dir):
        print(f"Error: Output path '{args.output_dir}' exists but is not a directory.", file=sys.stderr)
        sys.exit(1)
    
    # Validate input set
    valid_hd_sets = ['HD11', 'HD13', 'HD15', 'HD17', 'HD19', 'HD21', 'HD23']
    if args.input_set not in valid_hd_sets:
        print(f"Error: Input set '{args.input_set}' is not valid. Use one of: {valid_hd_sets}", file=sys.stderr)
        sys.exit(1)
    
    # Validate output set
    valid_prefixes = ('HC', 'HG', 'HO')
    if not any(args.output_set.startswith(prefix) for prefix in valid_prefixes):
        print(f"Error: Output set '{args.output_set}' must start with one of: {valid_prefixes}", file=sys.stderr)
        sys.exit(1)
    
    # Print configuration
    print("=" * 60)
    print("STag Marker Generator")
    print("=" * 60)
    print(f"Input Directory:  {args.input_dir}")
    print(f"Output Directory: {args.output_dir}")
    print(f"Input Set:        {args.input_set}")
    print(f"Output Set:       {args.output_set}")
    if args.sample_size > 0:
        print(f"Sample Size:      {args.sample_size}")
    print("=" * 60)
    print()
    
    # Generate markers based on output set type
    try:
        if args.output_set.startswith('HC'):
            print("Generating High-Capacity (HC) markers...")
            generate_high_capacity_markers(args.input_set, args.input_dir, args.output_dir)
            
        elif args.output_set.startswith('HG'):
            print("Generating Hue-Greyscale (HG) markers...")
            generate_hue_greyscale_markers(args.input_set, args.input_dir, args.output_dir)
            
        elif args.output_set.startswith('HO'):
            print("Generating High-Occlusion (HO) markers...")
            generate_high_occlusion_markers(
                args.input_set,
                args.input_dir,
                args.output_dir,
                sample_size=args.sample_size
            )
        
        print()
        print("=" * 60)
        print("Generation complete!")
        print(f"Markers saved to: {args.output_dir}")
        print("=" * 60)
        
    except Exception as e:
        print(f"\nError during marker generation: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
