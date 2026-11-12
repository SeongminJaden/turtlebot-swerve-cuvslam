#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
py="$PWD/.venv-perception-live/bin/python"
run="$PWD/runs/perception/train-v11-domain"
data="$PWD/runs/perception/data-v11-domain"
[[ "$(cat "$run/status.txt")" == TRAINED_AND_TESTED ]]
comparison="$run/baseline-v9-on-new-test"
mkdir -p "$comparison/yolo/weights" "$comparison/centerpose"
ln -s "$PWD/runs/perception/train-v9-accuracy/yolo/weights/best.pt" "$comparison/yolo/weights/best.pt"
ln -s "$PWD/runs/perception/train-v9-accuracy/centerpose/best.pt" "$comparison/centerpose/best.pt"
"$py" perception/scripts/evaluate_models.py --data "$data" --run "$comparison" --split test --imgsz 800 --pose-views scale3_median > "$run/baseline-test.log" 2>&1
for version in v9 v11; do
  weights="$PWD/runs/perception/train-v9-accuracy/yolo/weights/best.pt"
  [[ "$version" == v9 ]] || weights="$run/yolo/weights/best.pt"
  "$py" perception/scripts/evaluate_box_deformation.py --data "$data" --weights "$weights" --output "$run/box-deformation-$version.json" > "$run/box-deformation-$version.log" 2>&1
done
"$py" perception/scripts/compare_real_frame.py --reference "$run/real-reference.json" --weights "$run/yolo/weights/best.pt" --output "$run/real-final" > "$run/real-final.log" 2>&1
printf 'COMPARISONS_COMPLETE\n' > "$run/comparison-status.txt"
