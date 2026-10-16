"""Parser for JSONL log files containing marker detection data."""

import json
from pathlib import Path
from typing import Optional
from ..models.log_data import LogData, FrameData


class LogParser:
    """Parser for JSONL format log files."""
    
    @staticmethod
    def parse(file_path: Path) -> Optional[LogData]:
        """
        Parse a JSONL log file.
        
        Args:
            file_path: Path to the JSONL file
            
        Returns:
            LogData object or None if parsing fails
        """
        if not file_path.exists():
            print(f"Warning: Log file not found: {file_path}")
            return None
        
        frames = []
        log_version = "1.0"
        created_at = ""
        total_frames = 0
        
        try:
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    
                    # Skip empty lines
                    if not line:
                        continue
                    
                    # Handle comment lines (metadata)
                    if line.startswith('//'):
                        # Remove '//' and parse JSON
                        json_str = line[2:].strip()
                        try:
                            metadata = json.loads(json_str)
                            
                            # Extract metadata
                            if 'log_type' in metadata:
                                if metadata['log_type'] == 'stag_detection_log':
                                    log_version = metadata.get('version', '1.0')
                                    created_at = metadata.get('created_at', '')
                                elif metadata['log_type'] == 'stag_detection_log_end':
                                    total_frames = metadata.get('total_frames', 0)
                        except json.JSONDecodeError:
                            pass  # Skip malformed metadata
                        continue
                    
                    # Parse frame data
                    try:
                        frame_data = json.loads(line)
                        frame = FrameData(
                            timestamp=frame_data['timestamp'],
                            frame_id=frame_data['frame_id'],
                            marker_ids=frame_data['marker_ids']
                        )
                        frames.append(frame)
                    except (json.JSONDecodeError, KeyError) as e:
                        print(f"Warning: Skipping malformed line in {file_path}: {e}")
                        continue
            
            # If total_frames wasn't in metadata, use actual count
            if total_frames == 0:
                total_frames = len(frames)
            
            return LogData(
                file_path=str(file_path),
                frames=frames,
                log_version=log_version,
                created_at=created_at,
                total_frames=total_frames
            )
            
        except Exception as e:
            print(f"Error parsing log file {file_path}: {e}")
            return None
