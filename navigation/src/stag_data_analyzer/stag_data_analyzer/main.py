"""Main entry point for STag data analysis."""

import argparse
from pathlib import Path
import sys
from typing import Dict

from stag_data_analyzer.core.config_manager import ConfigManager
from stag_data_analyzer.parsers.path_parser import PathParser
from stag_data_analyzer.analyzers.statistics_calculator import StatisticsCalculator
from stag_data_analyzer.visualizers.heatmap_generator import HeatmapGenerator
from stag_data_analyzer.analyzers.comparators.marker_size_comparator import MarkerSizeComparator
from stag_data_analyzer.visualizers.comparison_visualizer import ComparisonVisualizer


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Analyze STag marker detection experiment data'
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='Path to configuration YAML file'
    )
    
    parser.add_argument(
        '--summary-only',
        action='store_true',
        help='Only print dataset summary without generating any output'
    )
    
    parser.add_argument(
        '--mode',
        type=str,
        choices=['individual', 'comparison', 'all'],
        default=None,
        help='Override mode: individual (only heatmaps), comparison (only comparisons), all (both)'
    )
    
    return parser.parse_args()


def main():
    """Main entry point for the analyzer."""
    print("\n" + "="*60)
    print("STag Data Analyzer")
    print("="*60 + "\n")
    
    args = parse_arguments()
    
    try:
        if args.config:
            config_path = Path(args.config)
        else:
            # Try to find default config
            # First, try relative to current directory
            config_path = Path.cwd() / 'src' / 'stag_data_analyzer' / 'config' / 'analysis_config.yaml'
            
            if not config_path.exists():
                # Try in the package directory
                config_path = Path(__file__).parent.parent / 'config' / 'analysis_config.yaml'
            
            if not config_path.exists():
                print("Error: No configuration file found.")
                print("Please specify --config path/to/config.yaml")
                sys.exit(1)
        
        config_manager = ConfigManager(config_path)
        
    except Exception as e:
        print(f"Error loading configuration: {e}")
        sys.exit(1)
    
    paths = config_manager.get_paths()
    gathered_data_path = config_manager.resolve_path(paths['gathered_data'])
    output_path = config_manager.resolve_path(paths['output_directory'])
    
    print(f"📂 Data directory: {gathered_data_path}")
    print(f"📂 Output directory: {output_path}\n")
    
    # Check if data directory exists
    if not gathered_data_path.exists():
        print(f"Error: Data directory not found: {gathered_data_path}")
        sys.exit(1)
    
    # Parse dataset
    print("🔍 Parsing dataset structure...")
    path_parser = PathParser()
    dataset = path_parser.parse_dataset(
        gathered_data_path,
        config_manager.get_marker_sizes(),
        load_logs=True
    )
    
    # Print dataset summary
    PathParser.print_dataset_summary(dataset)
    
    if args.summary_only:
        print("Summary only mode - exiting without generating any output.")
        return
    
    # Determine which modes to run
    modes_config = config_manager.get_modes()
    
    # Command line overrides config
    if args.mode:
        run_individual = args.mode in ['individual', 'all']
        run_comparison = args.mode in ['comparison', 'all']
    else:
        run_individual = modes_config.get('individual_heatmaps', True)
        run_comparison = modes_config.get('compare_by_marker_size', False)
    
    print(f"\n🔧 Active modes:")
    print(f"  - Individual heatmaps: {'✓' if run_individual else '✗'}")
    print(f"  - Comparisons: {'✓' if run_comparison else '✗'}\n")
    
    # Initialize components
    stats_calculator = StatisticsCalculator()
    heatmap_generator = HeatmapGenerator(config_manager.config) if run_individual else None
    
    # Process individual experiments if enabled
    if run_individual:
        print("\n🔄 Processing individual experiments...\n")
        experiments_processed, experiments_failed = process_individual_experiments(
            dataset,
            stats_calculator,
            heatmap_generator,
            output_path
        )
    else:
        experiments_processed = 0
        experiments_failed = 0
    
    # Process comparisons if enabled
    if run_comparison:
        comparisons_count = process_comparisons(
            config_manager,
            dataset,
            output_path
        )
    else:
        comparisons_count = 0
    
    # Final summary
    print("\n" + "="*60)
    print("Analysis Complete!")
    print("="*60)
    if run_individual:
        print(f"✓ Experiments processed: {experiments_processed}")
        if experiments_failed > 0:
            print(f"❌ Experiments failed: {experiments_failed}")
        print(f"📂 Individual results: {output_path}")
    if run_comparison:
        print(f"✓ Comparisons processed: {comparisons_count}")
        print(f"📂 Comparison results: {output_path / 'comparisons'}")
    print("="*60 + "\n")


def process_individual_experiments(
    dataset,
    stats_calculator,
    heatmap_generator,
    output_path: Path
):
    """
    Process individual experiment heatmaps.
    
    Args:
        dataset: Parsed dataset
        stats_calculator: Statistics calculator
        heatmap_generator: Heatmap generator
        output_path: Output directory
        
    Returns:
        Tuple of (experiments_processed, experiments_failed)
    """
    experiments_processed = 0
    experiments_failed = 0
    
    for size_code in dataset.get_size_codes():
        size_group = dataset.get_marker_size(size_code)
        
        for marker_type_name in size_group.get_marker_type_names():
            marker_type = size_group.get_marker_type(marker_type_name)
            
            for exp_name in marker_type.get_experiment_names():
                experiment = marker_type.get_experiment(exp_name)
                
                # Skip if no data
                if not experiment.has_data():
                    print(f"⚠️  Skipping {size_code}/{marker_type.full_name}/{exp_name} - no data")
                    continue
                
                try:
                    # Calculate statistics
                    stats = stats_calculator.calculate_experiment_statistics(experiment)
                    
                    # Generate output path
                    output_file = (
                        output_path /
                        size_code /
                        marker_type.directory_name /
                        exp_name /
                        f"{exp_name}_heatmap.png"
                    )
                    
                    # Generate heatmap
                    heatmap_generator.generate_heatmap(
                        statistics=stats,
                        marker_size=size_group.display_name,
                        marker_type=marker_type.full_name,
                        experiment_name=exp_name,
                        output_path=output_file
                    )
                    
                    experiments_processed += 1
                    
                except Exception as e:
                    print(f"❌ Error processing {size_code}/{marker_type.full_name}/{exp_name}: {e}")
                    experiments_failed += 1
    
    return experiments_processed, experiments_failed


def process_comparisons(
    config_manager: ConfigManager,
    dataset,
    output_path: Path
) -> int:
    """
    Process comparison analysis.
    
    Args:
        config_manager: Configuration manager
        dataset: Parsed dataset
        output_path: Output directory
        
    Returns:
        Number of comparisons processed
    """
    print("\n" + "="*60)
    print("Processing Marker Size Comparisons")
    print("="*60 + "\n")
    
    comparisons = config_manager.get_marker_size_comparisons()
    
    if not comparisons:
        print("⚠️  No marker size comparisons configured.")
        return 0
    
    comparisons_count = 0
    
    for comp_config in comparisons:
        count = process_single_marker_type_comparison(
            config_manager,
            dataset,
            output_path,
            comp_config
        )
        comparisons_count += count
    
    return comparisons_count


def process_single_marker_type_comparison(
    config_manager: ConfigManager,
    dataset,
    output_path: Path,
    comp_config: Dict
) -> int:
    """
    Process comparisons for a single marker type.
    
    Args:
        config_manager: Configuration manager
        dataset: Parsed dataset
        output_path: Output directory
        comp_config: Single comparison configuration
            {
                'marker_type': 'HD17_vert',
                'sizes': ['m40', 'm80', 'm100'],
                'experiments': ['light_manip', ...]  # optional
            }
            
    Returns:
        Number of comparisons processed
    """
    marker_type = comp_config.get('marker_type', '')
    sizes = comp_config.get('sizes', [])
    specific_experiments = comp_config.get('experiments', None)
    
    if not marker_type or len(sizes) < 2:
        print(f"⚠️  Invalid comparison config: {comp_config}")
        return 0
    
    print(f"� Marker Type: {marker_type}")
    print(f"   Sizes: {', '.join(sizes)}")
    
    # Find common experiments between all specified sizes
    common_experiments = find_common_experiments(dataset, marker_type, sizes)
    
    # Filter to specific experiments if provided
    if specific_experiments:
        common_experiments = [exp for exp in common_experiments if exp in specific_experiments]
    
    if not common_experiments:
        print(f"   ⚠️  No common experiments found for sizes: {sizes}\n")
        return 0
    
    print(f"   Common experiments: {', '.join(common_experiments)}\n")
    
    # Initialize components
    comparator = MarkerSizeComparator(dataset)
    visualizer = ComparisonVisualizer(config_manager.config)
    viz_config = config_manager.get_comparison_visualization_config()
    total_markers = config_manager.get('markers.total_count', 70)
    
    comparisons_count = 0
    
    # Process each common experiment
    for experiment_name in common_experiments:
        print(f"   🧪 {experiment_name}")
        
        try:
            # Get experiments for all sizes at once
            experiments_dict = comparator.get_experiments_for_comparison(
                marker_type,
                experiment_name,
                sizes
            )
            
            # Calculate metrics for all sizes
            metrics = comparator.compare_experiments(
                marker_type,
                experiment_name,
                sizes,
                total_markers
            )
            
            # Create output directory
            sizes_str = '_'.join(sizes)
            comparison_output = (
                output_path / "comparisons" / 
                marker_type / 
                experiment_name
            )
            comparison_output.mkdir(parents=True, exist_ok=True)
            
            # Generate visualizations
            visualizations = viz_config.get('visualizations', [])
            output_formats = viz_config.get('output_formats', ['html', 'png'])
            
            if 'side_by_side_heatmaps' in visualizations:
                base_name = f"{sizes_str}_heatmaps"
                visualizer.generate_side_by_side_heatmaps(
                    experiments_dict,
                    marker_type,
                    experiment_name,
                    comparison_output / base_name,
                    sizes,
                    output_formats
                )
            
            if 'metrics_table' in visualizations:
                base_name = f"{sizes_str}_metrics"
                visualizer.generate_metrics_table(
                    metrics,
                    marker_type,
                    experiment_name,
                    comparison_output / base_name,
                    sizes,
                    output_formats
                )
            
            print(f"      ✓ {' + '.join(sizes)}")
            comparisons_count += 1
            
        except Exception as e:
            print(f"      ❌ Error: {e}")
            import traceback
            traceback.print_exc()
    
    print()
    return comparisons_count


def find_common_experiments(dataset, marker_type: str, sizes: list) -> list:
    """
    Find experiments that exist across all specified marker sizes.
    
    Args:
        dataset: Parsed dataset
        marker_type: Marker type name (e.g., 'HD17_vert')
        sizes: List of size codes (e.g., ['m40', 'm80', 'm100'])
        
    Returns:
        List of common experiment names
    """
    if not sizes:
        return []
    
    # Get experiments for first size
    try:
        first_size_group = dataset.get_marker_size(sizes[0])
        first_marker_type = first_size_group.get_marker_type(marker_type)
        common_exps = set(first_marker_type.get_experiment_names())
    except (KeyError, AttributeError):
        return []
    
    # Intersect with experiments from other sizes
    for size in sizes[1:]:
        try:
            size_group = dataset.get_marker_size(size)
            marker_type_group = size_group.get_marker_type(marker_type)
            size_exps = set(marker_type_group.get_experiment_names())
            common_exps &= size_exps
        except (KeyError, AttributeError):
            return []
    
    return sorted(list(common_exps))


def get_size_display(config_manager: ConfigManager, size_code: str) -> str:
    """
    Get display name for a size code.
    
    Args:
        config_manager: Configuration manager
        size_code: Size code (e.g., 'm40')
        
    Returns:
        Display name (e.g., '40x40 мм')
    """
    marker_sizes = config_manager.get_marker_sizes()
    for size_config in marker_sizes:
        if size_config.get('name') == size_code:
            return size_config.get('display', size_code)
    return size_code


if __name__ == '__main__':
    main()
