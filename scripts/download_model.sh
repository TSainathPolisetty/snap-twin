#!/bin/bash
# Download Depth Anything V2 Small ONNX model
# Run once on the Jetson Orin before converting with scripts/convert_model.sh

set -e

mkdir -p ~/models

echo "Installing dependencies..."
pip3 install huggingface_hub --break-system-packages

echo "Downloading Depth Anything V2 Small ONNX..."
python3 - << 'EOF'
from huggingface_hub import hf_hub_download
import shutil, os

print("Downloading Depth Anything V2 Small ONNX...")
path = hf_hub_download(
    repo_id="depth-anything/Depth-Anything-V2-Small",
    filename="depth_anything_v2_vits.onnx",
    cache_dir="/tmp/da_cache"
)

dest = "/home/ubuntu/models/depth_anything_v2_small.onnx"
shutil.copy(path, dest)
print(f"Saved to {dest} ({os.path.getsize(dest)//1024//1024}MB)")
EOF

echo ""
echo "Done. Now run scripts/convert_model.sh to build the TensorRT engine."
