# STag Detector with Logging

Launch STag marker detector with automatic detection logging and optional rosbag recording.

## 🎯 Key Features

- ✅ STag marker detection (HD, HG, HC, HO types)
- ✅ Automatic detection logging in JSONL format
- ✅ Rosbag recording of image topics
- ✅ Flexible path configuration for orchestrator integration
- ✅ Graceful shutdown of all processes

## 🚀 Quick Start

### Basic Launch

```bash
ros2 launch stag_ros2_enhanced stag_detector_with_logging.launch.py
```

Default paths (launch argument defaults):
- Detection logs: `/home/app/ros2_ws/src/logging_files/output/logs/detections.jsonl`
- Rosbag (only with `enable_rosbag:=true`): `/home/app/ros2_ws/src/logging_files/output/bags/stag_experiment`

### Launch with Custom Paths (for Orchestrator)

```bash
ros2 launch stag_ros2_enhanced stag_detector_with_logging.launch.py \
    log_file_path:=/data/experiments/session_001/detections.jsonl \
    rosbag_output_dir:=/data/experiments/session_001/bags \
    rosbag_name:=exp_001_markers \
    enable_rosbag:=true \
    rosbag_topics:=/torso_rgbd/image_raw
```

### Logging and Rosbag

Detection logging is always on with this launch file. Rosbag recording is off by default:

```bash
# Logging + rosbag (simulation camera topic)
ros2 launch stag_ros2_enhanced stag_detector_with_logging.launch.py \
    enable_rosbag:=true \
    rosbag_topics:=/torso_rgbd/image_raw
```

For detection without logging use `stag_detector.launch.py`.

## 📋 Launch Parameters

| Parameter | Description | Default |
|----------|----------|--------------|
| `detector_config` | Path to detector YAML config | `<pkg_share>/config/detector_params_HD15.yaml` |
| `logger_config` | Path to logger YAML config | `<pkg_share>/config/logger_params.yaml` |
| `namespace` | Namespace for nodes | `stag_detector` |
| `log_file_path` | Path to JSONL log file | `/home/app/ros2_ws/src/logging_files/output/logs/detections.jsonl` |
| `marker_type` | Marker type (HD17, HC21, HG17, ...); overrides the YAML value if set | `''` |
| `enable_rosbag` | Enable rosbag recording | `false` |
| `rosbag_output_dir` | Directory for rosbag output | `/home/app/ros2_ws/src/logging_files/output/bags` |
| `rosbag_name` | Rosbag name | `stag_experiment` |
| `rosbag_topics` | Topics to record (space-separated) | `/camera/camera/color/image_raw` (real-robot RealSense topic) |

In simulation pass `rosbag_topics:=/torso_rgbd/image_raw`.

Logger options such as `include_metadata` and `auto_create_dirs` are node parameters, set in the `logger_config` YAML.

## 📊 JSONL Log File Format

### Record Structure

Each line is a JSON object with data from one detection tick:

```json
{"timestamp": 1729692345.123456, "frame_id": 0, "marker_ids": [0, 5, 12]}
{"timestamp": 1729692345.156789, "frame_id": 1, "marker_ids": [0, 5]}
{"timestamp": 1729692345.189012, "frame_id": 2, "marker_ids": []}
{"timestamp": 1729692345.223456, "frame_id": 3, "marker_ids": [0, 5, 12, 15, 20]}
```

### Fields

- `timestamp` (float): ROS timestamp in seconds (nanosecond precision)
- `frame_id` (int): Sequential frame number
- `marker_ids` (array of int): List of detected marker IDs

### With Metadata (`include_metadata: true` in the `logger_config` YAML)

```json
{
  "timestamp": 1729692345.123456,
  "frame_id": 0,
  "marker_ids": [0, 5, 12],
  "num_markers": 3
}
```

Additional fields:
- `num_markers`: Number of detected markers (len(marker_ids))

## 🐍 Reading JSONL Logs (Python)

### Basic Reading

```python
import json

detections = []
with open('detections.jsonl', 'r') as f:
    for line in f:
        # Skip comments
        if line.startswith('//'):
            continue
        
        data = json.loads(line)
        detections.append(data)

print(f"Total frames: {len(detections)}")
```

### Calculate FPS

```python
import json

timestamps = []
with open('detections.jsonl', 'r') as f:
    for line in f:
        if line.startswith('//'):
            continue
        data = json.loads(line)
        timestamps.append(data['timestamp'])

# Average FPS
if len(timestamps) > 1:
    total_time = timestamps[-1] - timestamps[0]
    avg_fps = len(timestamps) / total_time
    print(f"Average FPS: {avg_fps:.2f}")

# Instantaneous FPS for each frame
fps_list = []
for i in range(1, len(timestamps)):
    dt = timestamps[i] - timestamps[i-1]
    fps = 1.0 / dt if dt > 0 else 0
    fps_list.append(fps)

print(f"Max FPS: {max(fps_list):.2f}")
print(f"Min FPS: {min(fps_list):.2f}")
```

### Detection Analysis

```python
import json
from collections import Counter

marker_counter = Counter()
frames_with_markers = 0

with open('detections.jsonl', 'r') as f:
    for line in f:
        if line.startswith('//'):
            continue
        
        data = json.loads(line)
        marker_ids = data['marker_ids']
        
        if marker_ids:
            frames_with_markers += 1
            marker_counter.update(marker_ids)

print(f"Frames with markers: {frames_with_markers}")
print(f"Most common markers: {marker_counter.most_common(5)}")
```

### Pandas Analysis

```python
import json
import pandas as pd

# Чтение в DataFrame
data = []
with open('detections.jsonl', 'r') as f:
    for line in f:
        if not line.startswith('//'):
            data.append(json.loads(line))

df = pd.DataFrame(data)

# Add column with marker count
df['num_detected'] = df['marker_ids'].apply(len)

# Statistics
print(df.describe())

# FPS
df['dt'] = df['timestamp'].diff()
df['fps'] = 1.0 / df['dt']

print(f"\nFPS statistics:")
print(df['fps'].describe())

# Plot
import matplotlib.pyplot as plt

plt.figure(figsize=(12, 4))
plt.plot(df['timestamp'], df['num_detected'])
plt.xlabel('Time (sec)')
plt.ylabel('Number of markers')
plt.title('Marker Detection Over Time')
plt.grid(True)
plt.show()
```

## 🔧 Orchestrator Integration

### Python Orchestrator Example

```python
import subprocess
import os
from datetime import datetime

class STagOrchestrator:
    def __init__(self, base_dir="/data/experiments"):
        self.base_dir = base_dir
        self.session_id = None
        self.process = None
    
    def start_session(self, session_name=None):
        """Start a new detection session"""
        # Create session directory
        if session_name is None:
            session_name = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        self.session_id = session_name
        session_dir = os.path.join(self.base_dir, session_name)
        os.makedirs(session_dir, exist_ok=True)
        
        # Paths for logs and rosbag
        log_file = os.path.join(session_dir, "detections.jsonl")
        bag_dir = os.path.join(session_dir, "bags")
        bag_name = f"{session_name}_markers"
        
        # Launch the detector
        cmd = [
            'ros2', 'launch', 'stag_ros2_enhanced', 
            'stag_detector_with_logging.launch.py',
            f'log_file_path:={log_file}',
            f'rosbag_output_dir:={bag_dir}',
            f'rosbag_name:={bag_name}',
            'enable_rosbag:=true'
        ]
        
        self.process = subprocess.Popen(cmd)
        print(f"Session {session_name} started (PID: {self.process.pid})")
        print(f"  Logs: {log_file}")
        print(f"  Rosbag: {bag_dir}/{bag_name}")
        
        return session_dir
    
    def stop_session(self):
        """Stop the current session"""
        if self.process:
            self.process.terminate()
            self.process.wait(timeout=10)
            print(f"Session {self.session_id} stopped")
            self.process = None
            self.session_id = None

# Usage
orchestrator = STagOrchestrator()

# Start
session_dir = orchestrator.start_session("experiment_001")

# ... run experiment ...

# Stop
orchestrator.stop_session()
```

### Bash Orchestrator Script

```bash
#!/bin/bash

# Configuration
BASE_DIR="/data/experiments"
SESSION_NAME="session_$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${BASE_DIR}/${SESSION_NAME}"

# Create directories
mkdir -p "${SESSION_DIR}"

# Paths
LOG_FILE="${SESSION_DIR}/detections.jsonl"
BAG_DIR="${SESSION_DIR}/bags"
BAG_NAME="${SESSION_NAME}_markers"

# Launch
echo "Starting session: ${SESSION_NAME}"
echo "  Logs: ${LOG_FILE}"
echo "  Rosbag: ${BAG_DIR}/${BAG_NAME}"

ros2 launch stag_ros2_enhanced stag_detector_with_logging.launch.py \
    log_file_path:="${LOG_FILE}" \
    rosbag_output_dir:="${BAG_DIR}" \
    rosbag_name:="${BAG_NAME}" \
    enable_rosbag:=true

echo "Session completed"
```

## 📁 Data Structure After Launch

```
/data/experiments/
└── session_20251023_143025/
    ├── detections.jsonl          # Detection log
    └── bags/
        └── session_20251023_143025_markers/
            ├── metadata.yaml
            └── session_20251023_143025_markers_0.db3
```

## 🛑 Graceful Shutdown

All processes gracefully terminate on:
- `Ctrl+C` in terminal
- Launch file termination
- Orchestrator stop signal

ROS2 automatically:
- Closes JSONL file (with footer entry)
- Stops rosbag recording
- Terminates all nodes

## ⚙️ Detector Configuration

To change detection parameters, edit `config/detector_params_HD15.yaml` or `config/detector_params_HD17.yaml`:

```yaml
/**:
  ros__parameters:
    image_topic: '/torso_rgbd/image_raw'
    marker_type: 'HC21'
    marker_size: 0.058
    show_detection: true
    # ... other parameters
```

Or specify custom config:

```bash
ros2 launch stag_ros2_enhanced stag_detector_with_logging.launch.py \
    detector_config:=/path/to/custom_config.yaml
```

## 📝 Usage Examples

### 1. Test Launch

```bash
# Launch with default settings
ros2 launch stag_ros2_enhanced stag_detector_with_logging.launch.py
```

### 2. Multi-Camera Experiment

```bash
# Camera 1
ros2 launch stag_ros2_enhanced stag_detector_with_logging.launch.py \
    namespace:=camera1 \
    log_file_path:=/data/exp1/camera1_detections.jsonl \
    rosbag_output_dir:=/data/exp1/camera1_bags

# Camera 2 (in another terminal)
ros2 launch stag_ros2_enhanced stag_detector_with_logging.launch.py \
    namespace:=camera2 \
    log_file_path:=/data/exp1/camera2_detections.jsonl \
    rosbag_output_dir:=/data/exp1/camera2_bags
```

### 3. Visualization Only (No Recording)

This launch file always logs; use the plain detector launch instead:

```bash
ros2 launch stag_ros2_enhanced stag_detector.launch.py
```

## 🐛 Troubleshooting

### Issue: Log file not created

**Solution**: Check directory permissions and ensure `auto_create_dirs: true` in the `logger_config` YAML (node parameter, default `true`)

### Issue: Rosbag not recording

**Solution**: Verify that topics exist:
```bash
ros2 topic list
```

### Issue: No detections

**Solution**: Check detector configuration and camera topic:
```bash
ros2 topic echo /torso_rgbd/image_raw --no-arr
```

## 📚 Additional Resources

- [Main README](README.md) - General package information
- [Marker Generation](scripts/README.md) - How to create STag markers
- [ROS2 Launch Tutorial](https://docs.ros.org/en/humble/Tutorials/Launch/Launch-Main.html)
