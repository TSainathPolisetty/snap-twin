#!/bin/bash
# Download Depth Anything V2 Small ONNX model
# Run once on the Rubik Pi before launching the demo

set -e

MODEL_DIR="/home/ubuntu/models"
MODEL_FILE="$MODEL_DIR/depth_anything_v2_small.onnx"

mkdir -p "$MODEL_DIR"

if [ -f "$MODEL_FILE" ]; then
    echo "Model already exists at $MODEL_FILE"
    exit 0
fi

echo "Installing dependencies..."
pip3 install onnxruntime huggingface_hub --break-system-packages

echo "Downloading Depth Anything V2 Small ONNX..."
python3 - << 'EOF'
from huggingface_hub import hf_hub_download
import shutil

path = hf_hub_download(
    repo_id="onnx-community/depth-anything-v2-small",
    filename="onnx/model.onnx",
    cache_dir="/tmp/da_cache"
)

shutil.copy(path, "/home/ubuntu/models/depth_anything_v2_small.onnx")
print(f"Model saved to /home/ubuntu/models/depth_anything_v2_small.onnx")
EOF

echo ""
echo "Done. Test inference speed with:"
echo "  python3 -c \""
echo "    import onnxruntime as ort, numpy as np, time"
echo "    s = ort.InferenceSession('/home/ubuntu/models/depth_anything_v2_small.onnx')"
echo "    x = np.random.rand(1,3,518,518).astype('float32')"
echo "    t = time.time()"
echo "    for _ in range(5): s.run(None, {s.get_inputs()[0].name: x})"
echo "    print(f'{5/(time.time()-t):.2f} fps')\"" 
