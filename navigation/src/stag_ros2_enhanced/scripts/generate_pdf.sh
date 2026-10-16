#!/bin/bash
# Generate printable PDF with STag markers
# Edit the parameters below for your needs

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(dirname "$SCRIPT_DIR")"

# ==============================================================================
# CONFIGURATION - Edit these parameters
# ==============================================================================

INPUT_DIR="$PACKAGE_ROOT/markers/HC13"
MARKER_SIZE=60  # Size in millimeters
OUTPUT_FILE=""  # Leave empty for auto-generated name
CONFIG_FILE="$PACKAGE_ROOT/config/pdf_generator_config.yaml"

# ==============================================================================
# Script execution
# ==============================================================================

echo "====================================="
echo "STag PDF Generator"
echo "====================================="
echo "Input:       $INPUT_DIR"
echo "Marker size: ${MARKER_SIZE}mm"
if [ -n "$OUTPUT_FILE" ]; then
  echo "Output:      $OUTPUT_FILE"
else
  echo "Output:      Auto-generated name"
fi
echo "====================================="
echo ""

# Build command
CMD="python3 $SCRIPT_DIR/generate_pdf.py \
  --input $INPUT_DIR \
  --marker-size $MARKER_SIZE"

# Add output file if specified
if [ -n "$OUTPUT_FILE" ]; then
  CMD="$CMD --output $OUTPUT_FILE"
fi

# Add config if exists
if [ -f "$CONFIG_FILE" ]; then
  CMD="$CMD --config $CONFIG_FILE"
fi

# Execute
eval $CMD
