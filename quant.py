import qai_hub
import numpy as np

# Upload fixed model
print("Uploading...")
model = qai_hub.upload_model(
    "/home/ubuntu/models/depth_anything_v2_small_fixed.onnx")
print(f"Uploaded: {model.model_id}")

# Submit compile job with quantization options
job = qai_hub.submit_compile_job(
    model=model,
    device=qai_hub.Device("Dragonwing RB3 Gen 2 Vision Kit"),
    input_specs={"image": ((1, 3, 518, 518), "float32")},
    options=" ".join([
        "--target_runtime qnn_context_binary",
        "--quantize_full_type w8a16",       # quantize weights -> int8; activations -> int16
        "--quantize_io",                    # quantize input/output tensors too
    ]),
)
print(f"Job: {job.job_id}")
print(f"Track: https://workbench.aihub.qualcomm.com/jobs/{job.job_id}/")
job.wait()

status = job.get_status()
print(f"Status: {status.code}")
if status.success:
    job.download_target_model(
        "/home/ubuntu/models/depth_anything_v2_qcs6490.bin")
    print("Done: /home/ubuntu/models/depth_anything_v2_qcs6490.bin")
else:
    print(f"Failed: {status.message}")
