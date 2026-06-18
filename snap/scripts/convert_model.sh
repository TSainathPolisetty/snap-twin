#!/bin/bash
# Converts Depth Anything V2 Small ONNX → TensorRT engine
# Run once after download_model.sh

set -e

ONNX=~/models/depth_anything_v2_small.onnx
ENGINE=~/models/depth_anything_v2_small.engine

if [ ! -f "$ONNX" ]; then
    echo "ONNX not found. Run scripts/download_model.sh first."
    exit 1
fi

if [ -f "$ENGINE" ]; then
    echo "Engine already exists at $ENGINE"
    echo "Delete it first if you want to reconvert."
    exit 0
fi

echo "Converting ONNX → TensorRT engine (takes 5-10 minutes)..."
trtexec \
  --onnx="$ONNX" \
  --saveEngine="$ENGINE" \
  --fp16 \
  --minShapes=pixel_values:1x3x518x518 \
  --optShapes=pixel_values:1x3x518x518 \
  --maxShapes=pixel_values:1x3x518x518 \
  2>&1 | tee ~/models/conversion.log

echo ""
echo "Engine saved to $ENGINE"
echo "Size: $(du -h $ENGINE | cut -f1)"
