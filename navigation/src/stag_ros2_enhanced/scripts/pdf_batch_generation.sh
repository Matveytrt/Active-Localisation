#!/bin/bash
# Batch generate PDFs for multiple marker folders and sizes
# Edit the parameters below for your needs

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(dirname "$SCRIPT_DIR")"

# ==============================================================================
# CONFIGURATION - Edit these parameters
# ==============================================================================

# List of marker folders to process (space-separated)
FOLDERS=(
  "$PACKAGE_ROOT/markers/HD13"
  "$PACKAGE_ROOT/markers/HD15"
  "$PACKAGE_ROOT/markers/HD17"
  "$PACKAGE_ROOT/markers/HD21"
)

# List of marker sizes in millimeters (space-separated)
SIZES=(
  40
  60
  80
  100
)

OUTPUT_DIR="$PACKAGE_ROOT/output/pdfs"
CONFIG_FILE="$PACKAGE_ROOT/config/pdf_generator_config.yaml"

# ==============================================================================
# Script execution
# ==============================================================================

echo "====================================="
echo "STag Batch PDF Generator"
echo "====================================="
echo "Folders: ${#FOLDERS[@]}"
for folder in "${FOLDERS[@]}"; do
  echo "  - $(basename $folder)"
done
echo "Sizes: ${SIZES[@]}"
echo "Output: $OUTPUT_DIR"
echo "====================================="
echo ""

# Build command
CMD="python3 $SCRIPT_DIR/pdf_batch_generation.py"

# Add folders
if [ ${#FOLDERS[@]} -gt 0 ]; then
  CMD="$CMD --folders ${FOLDERS[@]}"
fi

# Add sizes
if [ ${#SIZES[@]} -gt 0 ]; then
  CMD="$CMD --sizes ${SIZES[@]}"
fi

# Add output directory
if [ -n "$OUTPUT_DIR" ]; then
  CMD="$CMD --output-dir $OUTPUT_DIR"
fi

# Add config if exists
if [ -f "$CONFIG_FILE" ]; then
  CMD="$CMD --config $CONFIG_FILE"
fi

# Execute
eval $CMD
