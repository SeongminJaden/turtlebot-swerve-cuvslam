#!/usr/bin/env bash
# Train and select on validation only. Freeze checkpoints before a separate test run.
set -euo pipefail
cd "$(dirname "$0")/../.."
python_bin="${PERCEPTION_PYTHON:-/home/seongmin/Projects/yolov8-ubuntu24-practice/.venv-yolo/bin/python}"
data="$(realpath "${1:-runs/perception/data-v8-clean}")"
output="$(realpath -m "${2:-runs/perception/train-clean}")"
mkdir -p "$output"
exec 9>"$output/training.lock"
flock -n 9 || exit 2
[[ ! -e "$output/yolo" && ! -e "$output/centerpose" ]] || { echo 'Refusing to overwrite an existing training run' >&2; exit 2; }
trap 'printf "FAILED\n" > "$output/status.txt"' ERR
[[ -f "$data/COMPLETE" ]]
"$python_bin" perception/scripts/check_dataset.py "$data" --expected-splits 800 100 100 > "$output/dataset-check.log" 2>&1
printf 'YOLO_RUNNING\n' > "$output/status.txt"
"$python_bin" perception/scripts/train_yolo.py --data "$data/dataset.yaml" --output "$output" --model yolov8n-p2.yaml --weights "${PERCEPTION_YOLO_INIT:-/home/seongmin/Projects/yolov8-ubuntu24-practice/yolov8n.pt}" --epochs "${PERCEPTION_YOLO_EPOCHS:-60}" --batch 8 --imgsz 640 --target-map50 .97 --min-epochs 20 > "$output/yolo.log" 2>&1
cp "$output/yolo/weights/best.pt" "$output/yolo/weights/best-map50-95.pt"
cp "$output/yolo/weights/best-map50.pt" "$output/yolo/weights/best.pt"
printf 'CENTERPOSE_RUNNING\n' > "$output/status.txt"
pose_args=()
if [[ -n "${PERCEPTION_POSE_INIT:-}" ]]; then pose_args+=(--resume "$PERCEPTION_POSE_INIT"); fi
"$python_bin" perception/scripts/train_centerpose.py --manifest "$data/manifest.json" --output "$output/centerpose" --epochs "${PERCEPTION_POSE_EPOCHS:-80}" --batch 32 --lr "${PERCEPTION_POSE_LR:-.0003}" --cosine "${pose_args[@]}" > "$output/centerpose.log" 2>&1
printf 'VALIDATING\n' > "$output/status.txt"
"$python_bin" perception/scripts/evaluate_models.py --data "$data" --run "$output" --split val --imgsz 640 > "$output/validation.log" 2>&1
printf 'VALIDATION_READY\n' > "$output/status.txt"
