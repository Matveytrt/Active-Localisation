"""Configuration manager for loading and validating YAML config."""

import yaml
from pathlib import Path
from typing import Dict, Any, Optional


class ConfigManager:
    """Manager for loading and accessing configuration."""
    
    def __init__(self, config_path: Optional[Path] = None):
        """
        Initialize configuration manager.
        
        Args:
            config_path: Path to YAML configuration file.
                        If None, looks for default config.
        """
        self.config_path = config_path
        self.config: Dict[str, Any] = {}
        
        if config_path:
            self.load_config(config_path)
    
    def load_config(self, config_path: Path) -> None:
        """
        Load configuration from YAML file.
        
        Args:
            config_path: Path to YAML file
        """
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.config_path = config_path
        print(f"✓ Loaded configuration from: {config_path}")
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get configuration value by key.
        
        Args:
            key: Configuration key (supports dot notation, e.g., 'paths.output_directory')
            default: Default value if key not found
            
        Returns:
            Configuration value or default
        """
        keys = key.split('.')
        value = self.config
        
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        
        return value
    
    def get_paths(self) -> Dict[str, Path]:
        """
        Get paths configuration as Path objects.
        
        Returns:
            Dictionary of path configurations
        """
        paths_config = self.get('paths', {})
        return {
            key: Path(value) for key, value in paths_config.items()
        }
    
    def get_marker_sizes(self) -> list:
        """
        Get marker size configurations.
        
        Returns:
            List of marker size dictionaries
        """
        return self.get('marker_sizes', [])
    
    def get_markers_config(self) -> Dict:
        """Get markers configuration."""
        return self.get('markers', {})
    
    def get_visualization_config(self) -> Dict:
        """Get visualization configuration."""
        return self.get('visualization', {})
    
    def get_modes(self) -> Dict[str, bool]:
        """
        Get enabled/disabled modes.
        
        Returns:
            Dictionary with mode flags
        """
        return self.get('modes', {})
    
    def is_mode_enabled(self, mode: str) -> bool:
        """
        Check if a specific mode is enabled.
        
        Args:
            mode: Mode name (e.g., 'individual_heatmaps', 'compare_by_marker_size')
            
        Returns:
            True if mode is enabled, False otherwise
        """
        modes = self.get_modes()
        return modes.get(mode, False)
    
    def get_marker_size_comparisons(self) -> list:
        """
        Get marker size comparison configurations.
        
        Returns:
            List of comparison configurations:
            [
                {
                    'marker_type': 'HD17_vert',
                    'sizes': ['m40', 'm80', 'm100'],
                    'experiments': ['light_manip', ...]  # optional
                },
                ...
            ]
        """
        return self.get('marker_size_comparisons', [])
    
    def get_comparison_visualization_config(self) -> Dict:
        """
        Get comparison visualization settings.
        
        Returns:
            Dictionary with visualizations and output_formats
        """
        return self.get('comparison_visualization', {})
    
    def resolve_path(self, path: Path, relative_to: Optional[Path] = None) -> Path:
        """
        Resolve a path, making it absolute.
        
        Args:
            path: Path to resolve
            relative_to: Base path for relative paths (defaults to config directory)
            
        Returns:
            Absolute Path
        """
        if path.is_absolute():
            return path
        
        if relative_to is None:
            if self.config_path:
                relative_to = self.config_path.parent
            else:
                relative_to = Path.cwd()
        
        return (relative_to / path).resolve()
    
    def __repr__(self) -> str:
        return f"ConfigManager(config_path='{self.config_path}')"
