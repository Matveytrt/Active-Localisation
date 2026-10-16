# STag Experiment Manager

Orchestration system for automated STag marker detection experiments.

## Overview

This package provides a complete orchestration system for running automated experiments with the STag marker detector. It manages:

- **Experiment configuration** via YAML files
- **Automated path generation** for organized output storage
- **Keyboard-based control** for starting/stopping experiments
- **State management** with error handling and recovery
- **Subprocess-based launch** of detector and logger

## Features

- ✨ **Automated experiment series** - Run multiple marker types, experiment conditions, and attempts automatically
- 🎮 **Interactive keyboard control** - Start/stop experiments, pause series, retry failed attempts
- 📁 **Organized output structure** - Automatic directory creation with clear naming conventions
- 🔄 **State machine** - Robust state management with validation
- 🛠️ **HG marker support** - Special handling for HG markers with different darkness levels
- 🎯 **Dynamic marker type switching** - Automatically changes marker type for each experiment via launch argument
- 📊 **Progress tracking** - Real-time status updates in terminal

## Architecture

### Components

1. **OrchestratorNode** - Main orchestration node
   - Manages experiment queue
   - Launches/stops detector+logger subprocess
   - Handles state transitions
   - Publishes status updates

2. **KeyboardControlNode** - User input interface
   - Runs in separate terminal window
   - Reads keyboard commands
   - Publishes control messages

3. **ExperimentStateManager** - State machine
   - Manages experiment lifecycle states
   - Validates state transitions
   - Tracks experiment progress

4. **PathGenerator** - Path and filename generator
   - Creates organized directory structure
   - Handles HG marker special cases
   - Generates consistent naming

## Installation

### Prerequisites

- ROS 2 Humble
- Python 3.10+
- `stag_ros2_enhanced` package

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select stag_experiment_manager
source install/setup.bash
```

## Configuration

### Experiment Configuration File

Located at `config/experiment_config.yaml`:

```yaml
experiments:
  markers:
    - HD17
    # - HC21
    # - HG17_076-178

  experiment_types:
    - light   # Good lighting
    # - dark    # Low lighting
    # - cold    # Temperature variation

  gait_types:
    - slow_center_forward
    - medi_center_forward
    - diskr_center_forward
    - slow_center_side
    - medi_center_side
    - diskr_center_side

  attempts_per_combination: 1

output:
  base_path: "/home/app/data/res640/m40"
  bags_subdir: "bags"
  logs_subdir: "logs"

stag_detector:
  launch_file: "/home/app/ros2_ws/src/stag_ros2_enhanced/launch/stag_detector_with_logging.launch.py"
  detector_config: "/home/app/ros2_ws/src/stag_ros2_enhanced/config/detector_params_HD15.yaml"
  logger_config: "/home/app/ros2_ws/src/stag_ros2_enhanced/config/logger_params.yaml"
  namespace: "stag_detector"
  rosbag_topics: "/camera/camera/color/image_raw"

orchestrator:
  auto_create_dirs: true
  cooldown_time: 2.0
  verbose: true
```

`rosbag_topics` in the shipped config is the RealSense topic of the real robot (`/camera/camera/color/image_raw`). In simulation the camera topic is `/torso_rgbd/image_raw`.

### Output Directory Structure

```
output/
├── HC17/
│   ├── light_slow/
│   │   ├── 1/
│   │   │   ├── bags/
│   │   │   │   └── HC17_light_slow_1.db3
│   │   │   └── logs/
│   │   │       └── HC17_light_slow_1.jsonl
│   │   ├── 2/
│   │   └── 3/
│   ├── light_fast/
│   └── dark_slow/
├── HD17/
└── HG17_255-000/
```

### Filename Format

- **Pattern**: `{marker}_{experiment}_{gait}_{attempt}`
- **Example**: `HG17_255-000_light_slow_1.jsonl`

### HG Marker Handling

For HG markers with darkness variations:
- **Full ID in paths**: `HG17_255-000_light_slow_1`
- **Library name for detector**: `HG17` (extracted before underscore)
- **Automatic switching**: Orchestrator passes library name via `marker_type` launch argument

Example:
- Input marker: `HG17_255-000`
- Library parameter passed to detector: `HG17`
- Output filename: `HG17_255-000_light_slow_1.jsonl`

**Note**: The `marker_type` launch argument **overrides** the value in the detector YAML (e.g. `detector_params_HD15.yaml`), allowing the orchestrator to dynamically change marker types for each experiment without modifying configuration files.

## Usage

### Basic Workflow

1. **Configure experiments** in `experiment_config.yaml`

2. **Launch orchestrator** (in first terminal):
   ```bash
   ros2 launch stag_experiment_manager experiment_orchestrator.launch.py
   ```

3. **Launch keyboard control** (in second terminal):
   ```bash
   ros2 run stag_experiment_manager keyboard_control
   ```

4. **Run experiments**:
   - Press `SPACE` to start first experiment
   - Let it run (camera should be recording)
   - Press `SPACE` again to stop
   - Press `SPACE` to start next experiment
   - Repeat until all experiments complete

### Keyboard Commands

In the **Keyboard Control Terminal**:

| Key | Command | Description |
|-----|---------|-------------|
| `SPACE` | Start/Stop | Start or stop current experiment |
| `N` | Next | Skip to next experiment |
| `R` | Retry | Repeat current experiment |
| `P` | Pause | Pause entire experiment series |
| `Q` | Quit | Stop all experiments and exit |
| `S` | Skip | Skip current experiment (after error) |
| `H` | Help | Show keyboard commands |

### State Flow

```
IDLE → READY → RUNNING → READY → RUNNING → ... → COMPLETED
          ↓       ↓
        PAUSED  ERROR
```

**States**:
- `IDLE`: System not started
- `READY`: Ready to start next experiment
- `RUNNING`: Experiment currently recording
- `PAUSED`: Series paused by user
- `ERROR`: Error occurred, waiting for user action
- `COMPLETED`: All experiments finished

### Example Session

```
[INFO] EXPERIMENT ORCHESTRATOR INITIALIZED
[INFO] Total experiments: 45
[INFO] Press SPACE in the keyboard control terminal to start

>>> SPACE pressed <<<
[INFO] STARTING EXPERIMENT 1/45
[INFO] Marker:      HC17
[INFO] Experiment:  light
[INFO] Gait:        slow
[INFO] Attempt:     1
[INFO] Experiment RUNNING. Press SPACE to stop.

>>> SPACE pressed <<<
[INFO] STOPPING EXPERIMENT
[INFO] Ready for next: HC17_light_slow_2
[INFO] Press SPACE to start

>>> SPACE pressed <<<
[INFO] STARTING EXPERIMENT 2/45
...
```

## Advanced Usage

### Custom Configuration File

```bash
ros2 launch stag_experiment_manager experiment_orchestrator.launch.py \
    config_file:=/path/to/custom_config.yaml
```

### Monitoring Progress

Watch the orchestrator terminal for:
- Current experiment details
- Progress percentage
- State transitions
- Error messages

### Error Recovery

If the detector fails to launch (the process exits within ~1 s after start):

1. **Orchestrator enters ERROR state**
2. **Choose recovery action**:
   - Press `S` to skip the failed experiment (orchestrator moves to the next one and returns to READY)
   - Press `Q` to quit entire series

`R` has no effect in ERROR state, although the error log suggests it: retry is handled only in READY, where it just keeps the current experiment (start it with `SPACE`).

### Pausing Series

To take a break:
1. Wait until current experiment is in READY state
2. Press `P` to pause
3. When ready to resume, press `P` again

## ROS Topics

### Published

- `/experiment/control` (`std_msgs/String`) - Control commands from keyboard

### Internal Communication

- Orchestrator subscribes to `/experiment/control`
- Status updates logged to terminal (not published to topic)

## Troubleshooting

### "Detector process terminated immediately"

**Cause**: Launch file or parameters incorrect

**Solution**: 
- Check `stag_detector.launch_file` path in config
- Verify detector package is built and sourced
- Check detector configuration files exist

### "Failed to create directories"

**Cause**: Insufficient permissions or invalid base path

**Solution**:
- Check `output.base_path` in config is writable
- Verify directory permissions
- Set `auto_create_dirs: true` in config

### Keyboard control not responding

**Cause**: Terminal emulator not supported or not in focus

**Solution**:
- Make sure control terminal window has focus
- Try alternative terminal (xterm instead of gnome-terminal)

### HG marker library not found

**Cause**: Library name extraction failed

**Solution**:
- Ensure HG marker format is correct: `HG17_xxx-xxx`
- Check detector supports the library name: marker types come from the STag library and are selected via the `marker_type` launch argument (`stag_ros2_enhanced/markers/` only holds a zip of HD15 marker images, not detector library files)

## Development

### Package Structure

```
stag_experiment_manager/
├── config/
│   └── experiment_config.yaml
├── launch/
│   └── experiment_orchestrator.launch.py
├── stag_experiment_manager/
│   ├── __init__.py
│   ├── orchestrator_node.py
│   ├── keyboard_control_node.py
│   ├── experiment_state_manager.py
│   └── path_generator.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

### Adding New Experiment Types

1. Edit `experiment_config.yaml`
2. Add new entries to `experiment_types` or `gait_types`
3. No code changes needed - orchestrator auto-generates combinations

### Extending Keyboard Commands

1. Edit `keyboard_control_node.py`
2. Add new key mapping in `read_keyboard()` method
3. Define command constant
4. Update help text in `print_help()`

## License

MIT

## Author

Developed for automated STag marker detection experiments.

## See Also

- [stag_ros2_enhanced](../stag_ros2_enhanced/README.md) - STag detector package
- [stag_ros2_enhanced/LOGGING_README.md](../stag_ros2_enhanced/LOGGING_README.md) - Logger documentation
