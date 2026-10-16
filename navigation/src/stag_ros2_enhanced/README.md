# STag ROS2 Enhanced

Professional ROS2 package for STag fiducial marker detection and generation.

## Features

### Detection
- Support for all STag marker types: HD, HG, HC, HO
- Real-time marker detection from camera topics
- Optional visualization of detected markers
- **TF broadcasting for marker poses** (6DOF pose estimation using cv2.solvePnP)
- Marker tracking for stable detections
- Detection statistics (FPS, timing)
- RViz visualization support

### Generation
- Generate custom STag markers (HC, HG, HO types)
- Create printable PDF files with marker grids
- Batch generation support
- Configurable marker sizes and layouts

## Installation

```bash
cd ~/ros2_ws/src
# Package should be here as stag_ros2_enhanced
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
# NumPy must be < 2.0 for cv_bridge compatibility with ROS2 Humble
pip install "numpy<2.0" stag-python opencv-python pyyaml reportlab Pillow scipy
# Clean rebuild to ensure proper installation
rm -rf build/stag_ros2_enhanced install/stag_ros2_enhanced
colcon build --symlink-install --packages-select stag_ros2_enhanced
source install/setup.bash
```

## Usage

### Marker Detection

```bash
# Launch detector node
ros2 launch stag_ros2_enhanced stag_detector.launch.py

# Or run directly
ros2 run stag_ros2_enhanced stag_detector_node
```

### Marker Generation

Marker generation is available through standalone scripts (no ROS2 environment needed):

```bash
cd scripts/

# Generate markers (edit configuration in script)
./generate_markers.sh

# Generate PDF (edit configuration in script)
./generate_pdf.sh

# Batch generate PDFs (edit configuration in script)
./pdf_batch_generation.sh
```

See `scripts/README.md` for detailed usage instructions.

## Configuration

Edit `config/detector_params_HD15.yaml` or `config/detector_params_HD17.yaml` for production; `detector_params.yaml` is a generic legacy template.

## Markers Directory

The repository ships only `markers/HD15-20260309T085339Z-1-001.zip` (unpacks to `markers/HD15/`). Other sets must be downloaded (link below) or generated with the scripts. Expected layout once unpacked/generated:
```
markers/
├── HD13/
├── HD15/
├── HD17/
├── HD21/
├── HG17/
├── HC21/
└── HO17/
```
Download markers can be here [markers](https://drive.google.com/drive/folders/0ByNTNYCAhWbIRmNCbzN1bThUMFE?resourcekey=0-BQ2SNu7Ufi9eKRtAUxrQVQ)


## License

MIT License
