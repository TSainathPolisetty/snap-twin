#!/bin/bash
# Download Depth Anything V2 Small ONNX model
# Run once on the Jetson Orin before converting with scripts/convert_model.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
MODEL_DIR="$REPO_ROOT/models"
mkdir -p "$MODEL_DIR"

echo "Installing dependencies..."
pip3 install huggingface_hub

echo "Downloading Depth Anything V2 Small ONNX..."
MODEL_DEST="$MODEL_DIR/depth_anything_v2_small.onnx" python3 - << 'EOF'
import os, shutil
from huggingface_hub import hf_hub_download

print("Downloading Depth Anything V2 Small ONNX...")
path = hf_hub_download(
    repo_id="onnx-community/depth-anything-v2-small",
    filename="onnx/model.onnx",
    cache_dir="/tmp/da_cache"
)

dest = os.environ["MODEL_DEST"]
shutil.copy(path, dest)
print(f"Saved to {dest} ({os.path.getsize(dest)//1024//1024}MB)")
EOF

echo ""
echo "Done. Model saved to $MODEL_DIR/depth_anything_v2_small.onnx"
echo "Now run scripts/convert_model.sh to build the TensorRT engine."
