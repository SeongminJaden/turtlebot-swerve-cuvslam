#!/usr/bin/env bash
# Independent YOLO and pose fitting share the GPU; model selection uses val only.
set -euo pipefail
cd "$(dirname "$0")/../.."
py="$PWD/.venv-perception-live/bin/python"
data="$(realpath "${1:-runs/perception/data-v11-domain}")"
run="$(realpath -m "${2:-runs/perception/train-v11-domain}")"
mkdir -p "$run"
exec 9>"$run/training.lock"
flock -n 9 || exit 2
[[ ! -d "$run/yolo" && ! -d "$run/centerpose" ]] || exit 2
telemetry_pid='';yolo_pid='';pose_pid=''
cleanup() { for pid in "$telemetry_pid" "$yolo_pid" "$pose_pid"; do [[ -z "$pid" ]] || kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT
trap 'printf "FAILED\n" > "$run/status.txt"' ERR
[[ -f "$data/COMPLETE" ]]
"$py" perception/scripts/check_dataset.py "$data" --expected-splits 800 100 100 > "$run/dataset-check.log" 2>&1
mkdir -p "$run/source"
cp perception/scripts/{generate_dataset,box_appearance,scene_variation,train_yolo,train_centerpose,evaluate_models,check_dataset}.py "$run/source/"
cp config/perception/materials.json "$run/source/materials.json"
printf 'YOLO_AND_CENTERPOSE_TRAINING\n' > "$run/status.txt"
nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used,temperature.gpu,power.draw --format=csv -l 1 > "$run/gpu-telemetry.csv" & telemetry_pid=$!
"$py" perception/scripts/train_yolo.py --data "$data/dataset.yaml" --output "$run" --model runs/perception/train-v9-accuracy/yolo/weights/best.pt --epochs 40 --imgsz 800 --batch 8 --lr .0002 > "$run/yolo.log" 2>&1 & yolo_pid=$!
"$py" perception/scripts/train_centerpose.py --manifest "$data/manifest.json" --output "$run/centerpose" --resume runs/perception/train-v9-accuracy/centerpose/best.pt --epochs 40 --batch 32 --lr .00008 --cosine > "$run/centerpose.log" 2>&1 & pose_pid=$!
wait "$yolo_pid";yolo_pid=''
wait "$pose_pid";pose_pid=''
cp "$run/yolo/weights/best.pt" "$run/yolo/weights/best-map50-95.pt"
cp "$run/yolo/weights/best-map50.pt" "$run/yolo/weights/best.pt"
printf 'VALIDATING\n' > "$run/status.txt"
"$py" perception/scripts/evaluate_models.py --data "$data" --run "$run" --split val --imgsz 800 --pose-views scale3_median > "$run/validation.log" 2>&1
"$py" - "$run" "$data" <<'PY'
import json,sys,hashlib,datetime
from pathlib import Path
r,d=map(Path,sys.argv[1:]);cfg={'imgsz':800,'pose_views':'scale3_median','detector_confidence':.25,'pose_confidence':.25,'box_dimensions_m':[.205,.205,.07],'box_dimensions_confirmed':False,'selection_split':'val','nonrigid_boxes':'detection only; excluded from rigid-pose evaluation'}
(r/'inference.json').write_text(json.dumps(cfg,indent=2))
paths=[r/'yolo/weights/best.pt',r/'centerpose/best.pt',r/'inference.json',d/'manifest.json']+list((r/'source').glob('*'))
(r/'frozen-before-test.json').write_text(json.dumps({'timestamp':datetime.datetime.now(datetime.timezone.utc).isoformat(),'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}},indent=2))
PY
printf 'TESTING_FROZEN_MODELS\n' > "$run/status.txt"
"$py" perception/scripts/evaluate_models.py --data "$data" --run "$run" --split test --imgsz 800 --pose-views scale3_median > "$run/test-evaluation.log" 2>&1
printf 'TRAINED_AND_TESTED\n' > "$run/status.txt"
