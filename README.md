# Model Training — YoloLite for TurtleBot3 Obstacle Detection

This document covers everything needed to train, evaluate, and export the **YoloLite** model used by the `turtlebot_obstacle_detection` ROS 2 package.

The model is a custom-trained lightweight YOLO variant designed for **CPU-only real-time inference** at 320×320 resolution, targeting deployment on the TurtleBot3 Burger's onboard computer.

---

## Model Architecture

The project uses a custom `YOLOLiteMS_CPU` architecture defined in [`model_tree_13.yaml`](yololite/model_tree_13.yaml):

| Parameter | Value |
|-----------|-------|
| Architecture | `YOLOLiteMS_CPU` |
| Backbone | `mobilenetv4_conv_small_050` |
| Depth multiple | `0.65` |
| Width multiple | `0.6` |
| FPN channels | `160` |
| Head depth | `1` |
| Classes | `13` |
| Input resolution | `320 × 320` |

Available pre-built configs in `yololite/configs/models/`:

| Config | Parameters | Target |
|--------|-----------|--------|
| `edge_n` | ~0.55M | CPU real-time (Raspberry Pi, embedded) |
| `edge_s` | ~0.9M | Balanced CPU speed/accuracy |
| `edge_m` | ~2.95M | YOLOv5-level accuracy, CPU |
| `edge_l` / `edge_xl` | 4M+ | Higher accuracy, CPU acceptable |
| `yololite_n/s/m/l/xl` | variable | GPU-oriented, highest mAP |

---

## Dataset

### Classes (13-class COCO subset)

```
0: person         5: cat           10: backpack
1: bicycle        6: chair         11: suitcase
2: car            7: couch         12: tree
3: motorcycle     8: dining table
4: dog            9: potted plant
```

### Dataset Structure

```
dataset/
├── train/
│   ├── images/        # .jpg / .png training images
│   └── labels/        # YOLO-format .txt labels (one per image)
├── valid/
│   ├── images/
│   └── labels/
└── test/              # optional
    ├── images/
    └── labels/
```

Each label file (`<image_name>.txt`) contains one detection per line:
```
<class_id> <cx> <cy> <w> <h>   # all values normalized 0–1
```

### Preparing a COCO Subset

Use the included script to download and filter COCO images by class:

```bash
cd yololite
python prepare_coco_subset.py
```

---

## Setup

```bash
cd yololite
pip install -r requirements.txt
```

**Requirements:** `torch`, `torchvision`, `onnx`, `onnxruntime`, `opencv-python`, `timm`, `pycocotools`, `PyYAML`, `tqdm`

---

## Training

### Quick Start

```bash
cd yololite
python tools/train.py \
  --model model_tree_13.yaml \
  --train train_tree_13.yaml \
  --data dataset.yaml \
  --epochs 200 \
  --batch_size 16 \
  --device 0 \
  --img_size 320
```

### Training Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--model` | required | Path to model `.yaml` |
| `--train` | `configs/train/standard_train.yaml` | Path to training config `.yaml` |
| `--data` | required | Path to dataset `.yaml` |
| `--epochs` | `200` | Total training epochs |
| `--batch_size` | `16` | Batch size |
| `--device` | `0` | GPU device id or `"cpu"` |
| `--img_size` | `640` | Input image size |
| `--workers` | `4` | DataLoader workers |
| `--augment` | `True` | Enable data augmentation |
| `--use_p2` | `False` | Add P2 head (better small object detection) |
| `--use_p6` | `False` | Add P6 head (large object detection) |
| `--use_resize` | `False` | Resize instead of letterbox |
| `--resume` | `None` | Path to checkpoint to resume from |
| `--lr` | `None` | Override learning rate |
| `--save_every` | `25` | Save checkpoint every N epochs |
| `--save_by` | `AP` | Save best model by metric: `AP50`, `AP75`, `AP`, `AR`, `APS`, `APM`, `APL` |

### Training Config (`train_tree_13.yaml`)

Key settings used for this project:

```yaml
training:
  loss_type: simota       # Assignment strategy
  optimizer: adamw
  lr: 1e-3
  scheduler: cosine
  img_size: 320
  batch_size: 16
  epochs: 5               # increase for production (200+ recommended)
  ema: true               # Exponential Moving Average weights
  ema_decay: 0.995
  amp: false              # Mixed precision (set true for faster GPU training)
  augment: true
  device: 0
```

### Resuming Training

When resuming, lower the learning rate to avoid disrupting the learned weights:

```bash
python tools/train.py \
  --model "runs/train/1/merged_config" \
  --data "dataset.yaml" \
  --resume "runs/train/1/best_model_state.pt" \
  --lr 0.0001
```

---

## Custom Model

To use an alternative backbone (any model available in [timm](https://timm.fast.ai/)):

1. Copy `configs/custom/custom.yaml`
2. Set the fields:

```yaml
model:
  backbone: efficientnet_b0   # any timm backbone
  depth_multiple: 0.7
  width_multiple: 0.5
  fpn_channels: 256           # must be divisible by 8 and 32
  head_depth: 2
```

---

## Evaluation

After training, evaluate on unseen test images:

```bash
python tools/evaluate.py \
  --weights runs/train/1/best_model_state.pt \
  --test_folder dataset/test \
  --img_size 320 \
  --batch_size 8
```

| Argument | Description |
|----------|-------------|
| `--weights` | Path to trained `.pt` checkpoint |
| `--test_folder` | Folder with `images/` and `labels/` subfolders |
| `--img_size` | Override inference resolution (0 = use model metadata) |
| `--device` | `"0"` for GPU or `"cpu"` |
| `--batch_size` | Evaluation batch size |
| `--no_letterbox` | Use resize instead of letterbox padding |

See [`MODEL_EVALUATION.md`](yololite/MODEL_EVALUATION.md) and [`BENCHMARK.md`](yololite/BENCHMARK.md) for full accuracy and latency benchmarks.

---

## Inference

Test a trained model on single images or folders:

```bash
python tools/infer.py \
  --weights runs/train/1/best_model_state.pt \
  --img image1.jpg \
  --conf 0.25 \
  --iou 0.50
```

Output images are saved to `runs/infer/<n>/`.

| Argument | Description |
|----------|-------------|
| `--img` | Single image path |
| `--img_dir` | Folder of images |
| `--conf` | Confidence threshold |
| `--iou` | NMS IoU threshold |
| `--max_det` | Maximum detections per image |
| `--save_txt` | Save YOLO-format `.txt` predictions |

---

## Export to ONNX

The ROS 2 nodes require an ONNX model in `decoded` format (boxes decoded, NMS applied).

```bash
python export/export_onnx.py \
  --weights runs/train/1/best_model_state.pt \
  --img-size 320 \
  --device cpu \
  --simplify \
  --format decoded
```

Output is saved to `runs/export/<n>/model_decoded.onnx`.

| Argument | Default | Description |
|----------|---------|-------------|
| `--weights` | required | Path to `.pt` checkpoint |
| `--img-size` | `640` | Export image size |
| `--device` | `"cpu"` | `"cpu"` or GPU id |
| `--opset` | `17` | ONNX opset version |
| `--simplify` | flag | Run `onnxsim` after export |
| `--dynamic-batch` | flag | Dynamic batch dimension |
| `--format` | `decoded` | `raw` (per-level) or `decoded` (boxes + NMS output) |

---

## ONNX Inference & Speed Benchmark

Test inference throughput using the ONNX runtime directly:

```bash
python export/infer_onnx.py \
  --model runs/export/1/model_decoded.onnx \
  --img image1.jpg \
  --img_size 320 \
  --warmup 10 \
  --runs 50
```

Example output:
```
=== Inference timing (ms) ===
pre_ms    mean 24.27 | std 0.00 | p50 24.27 | p90 24.27 | p95 24.27
infer_ms  mean 58.78 | std 0.00 | p50 58.78 | p90 58.78 | p95 58.78
post_ms   mean  1.17 | std 0.00 | p50  1.17 | p90  1.17 | p95  1.17
total_ms  mean 84.23 | std 0.00 | p50 84.23 | p90 84.23 | p95 84.23
Throughput ≈ 11.87 img/s
```

| Argument | Description |
|----------|-------------|
| `--providers` | `cpu`, `cuda`, `tensorrt` |
| `--warmup` | Warm-up runs before timing |
| `--runs` | Timed inference runs |
| `--intra` | `intra_op_num_threads` (0 = auto) |
| `--inter` | `inter_op_num_threads` (0 = auto) |

---

## Notebook Training

A Jupyter notebook is available for interactive training with visualizations:

```bash
jupyter notebook yololite/YoloLite_custom_training.ipynb
```

The project-level notebook [`model_training.ipynb`](model_training.ipynb) contains the full custom training pipeline used for this project.

---

## Output Structure

```
yololite/
└── runs/
    ├── train/
    │   └── <n>/
    │       ├── best_model_state.pt     # Best checkpoint (by AP)
    │       ├── last_model_state.pt     # Latest checkpoint
    │       └── merged_config/          # Model + train config snapshot
    ├── export/
    │   └── <n>/
    │       ├── model_decoded.onnx      # Decoded ONNX (for ROS 2 nodes)
    │       └── model_raw.onnx          # Raw ONNX (per-level outputs)
    └── infer/
        └── <n>/                        # Inference output images
```

---

## Performance Summary

| Model | Params | mAP@0.5 | CPU Latency (320px) | FPS |
|-------|--------|---------|---------------------|-----|
| `edge_n` | 0.55M | ~0.62 | ~9 ms | 100–200+ |
| `edge_m` | 2.95M | ~YOLOv5 | ~24 ms | ~40 |
| `yololite_n` | variable | 0.69–0.70 | GPU-oriented | GPU |

`edge_n` at 320×320 is the recommended configuration for TurtleBot3 CPU-only deployment.

---

## References

- [YoloLite upstream README](yololite/README.md)
- [Benchmark results](yololite/BENCHMARK.md)
- [Model evaluation report](yololite/MODEL_EVALUATION.md)
- [Simulation & ROS2 package README](src/turtlebot_obstacle_detection/README.md)
