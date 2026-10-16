# Standalone Scripts

These scripts can be run **without ROS2 environment** as regular Python scripts.

## 📋 Requirements

Install Python dependencies:
```bash
pip install stag-python opencv-python numpy pyyaml reportlab Pillow scipy
```

## 🚀 Usage

### 1. Generate Markers

Generate HC, HG, or HO markers from HD marker sets:

```bash
# Generate High-Capacity markers
python3 scripts/generate_markers.py \
  --input-dir markers/HD17 \
  --output-dir output/HC17 \
  --input-set HD17 \
  --output-set HC17

# Generate Hue-Greyscale markers
python3 scripts/generate_markers.py \
  --input-dir markers/HD17 \
  --output-dir output/HG17 \
  --input-set HD17 \
  --output-set HG17

# Generate High-Occlusion markers (limited sample)
python3 scripts/generate_markers.py \
  --input-dir markers/HD17 \
  --output-dir output/HO17 \
  --input-set HD17 \
  --output-set HO17 \
  --sample-size 100
```

### 2. Generate PDF

Create printable PDF files with marker grids:

```bash
# Basic usage
python3 scripts/generate_pdf.py \
  --input markers/HD17 \
  --marker-size 50

# With custom output name
python3 scripts/generate_pdf.py \
  --input markers/HD17 \
  --marker-size 80 \
  --output custom_markers.pdf

# With custom config
python3 scripts/generate_pdf.py \
  --input markers/HG17 \
  --marker-size 50 \
  --config config/pdf_generator_config.yaml
```

### 3. Batch PDF Generation

Generate PDFs for multiple marker sets and sizes at once:

```bash
# Edit configuration in the script first
./pdf_batch_generation.sh

# Or use Python directly with arguments:
python3 pdf_batch_generation.py \
  --folders markers/HD13 markers/HD15 markers/HD17 \
  --sizes 40 60 80 100 \
  --output-dir output/pdfs \
  --config ../config/pdf_generator_config.yaml
```

## 📂 File Paths

All paths are relative to the **package root** (`stag_ros2_enhanced/`):

```
stag_ros2_enhanced/
├── scripts/              # You are here
│   ├── generate_markers.py
│   ├── generate_pdf.py
│   └── pdf_batch_generation.py
├── markers/              # Input marker images
│   ├── HD13/
│   ├── HD17/
│   └── ...
├── config/               # Configuration files
│   └── pdf_generator_config.yaml
└── output/               # Generated files (create if needed)
```

## 💡 Examples

**Complete workflow:**

```bash
# 1. Navigate to package directory
cd /path/to/stag_ros2_enhanced

# 2. Generate HC markers from HD17
python3 scripts/generate_markers.py \
  --input-dir markers/HD17 \
  --output-dir markers/HC17 \
  --input-set HD17 \
  --output-set HC17

# 3. Generate PDFs for printing
python3 scripts/generate_pdf.py \
  --input markers/HC17 \
  --marker-size 50

# 4. Batch generate multiple sizes
python3 scripts/pdf_batch_generation.py \
  --folders markers/HD17 markers/HC17 \
  --sizes 40 60 80 \
  --output-dir pdfs_for_printing
```

## 🔧 Troubleshooting

**ModuleNotFoundError:**
```bash
# Install missing dependencies
pip install stag-python reportlab Pillow
```

**Permission denied:**
```bash
# Make scripts executable
chmod +x scripts/*.py
```

**Can't find markers:**
```bash
# Check you're in the right directory
cd /path/to/stag_ros2_enhanced
ls markers/  # Should show HD13, HD17, etc.
```

## 🆚 Standalone vs ROS2

**Standalone scripts (these):**
- ✅ No ROS2 required
- ✅ Work in any Python environment
- ✅ Simple to use for marker generation only

**ROS2 commands:**
- ✅ Integrated with ROS2 ecosystem
- ✅ Better for robotics workflows
- ✅ Access to detector node

```bash
# ROS2 way (after colcon build)
ros2 run stag_ros2_enhanced generate_pdf.py --input markers/HD17 --marker-size 50

# Standalone way (no ROS2 needed)
python3 scripts/generate_pdf.py --input markers/HD17 --marker-size 50
```

Both do the same thing, choose what fits your workflow!
