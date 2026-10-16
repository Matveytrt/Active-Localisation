#!/bin/bash
# Generate STag markers from HD marker sets
# Edit the parameters below for your needs

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(dirname "$SCRIPT_DIR")"

# ==============================================================================
# CONFIGURATION - Edit these parameters
# ==============================================================================

INPUT_DIR="$PACKAGE_ROOT/markers/HD13"
OUTPUT_DIR="$PACKAGE_ROOT/markers/HC13"
INPUT_SET="HD13"
OUTPUT_SET="HC13"  # HC, HG, or HO
SAMPLE_SIZE=""     # Optional: number of markers (for HO only), leave empty for all

# ==============================================================================
# Script execution
# ==============================================================================

echo "====================================="
echo "STag Marker Generator"
echo "====================================="
echo "Input:  $INPUT_DIR"
echo "Output: $OUTPUT_DIR"
echo "Type:   $INPUT_SET -> $OUTPUT_SET"
echo "====================================="
echo ""

# Build command
CMD="python3 $SCRIPT_DIR/generate_markers.py \
  --input-dir $INPUT_DIR \
  --output-dir $OUTPUT_DIR \
  --input-set $INPUT_SET \
  --output-set $OUTPUT_SET"

# Add sample size if specified (for HO markers)
if [ -n "$SAMPLE_SIZE" ]; then
  CMD="$CMD --sample-size $SAMPLE_SIZE"
fi

# Execute
eval $CMD
