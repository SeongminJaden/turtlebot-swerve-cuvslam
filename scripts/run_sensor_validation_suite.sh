#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
while [[ ! -f evidence/2026-10-26/predictive-qualified-gui/finished ]]; do sleep 1; done
flock build/predictive-runtime.lock true
python3 -c "import json; assert json.load(open('evidence/2026-10-26/predictive-qualified-gui/report.json'))['validated_success']"
ROUTE_FILE=assets/maze/route_clearance_second.json CUVSLAM_TRACKING_MODE=2 USE_EKF=1 USE_AMCL=1 CHECKPOINT=runs/predictive_checkpoints/v5-final.pt DURATION=180 bash scripts/run_predictive_async.sh evidence/2026-10-26/predictive-qualified-second > runs/predictive-qualified-second.log 2>&1
python3 -c "import json; assert json.load(open('evidence/2026-10-26/predictive-qualified-second/report.json'))['validated_success']"
FAULT_IMU2_START=4 FAULT_IMU2_DURATION=2 CUVSLAM_TRACKING_MODE=2 USE_EKF=1 USE_AMCL=1 CHECKPOINT=runs/predictive_checkpoints/v5-final.pt DURATION=25 bash scripts/run_predictive_async.sh evidence/2026-10-26/predictive-fault-imu > runs/predictive-fault-imu.log 2>&1
python3 scripts/analyze_sensor_fault.py evidence/2026-10-26/predictive-fault-imu --kind imu2 --start 4 --duration 2
FAULT_CAMERA_START=4 FAULT_CAMERA_DURATION=2.5 CUVSLAM_TRACKING_MODE=2 USE_EKF=1 USE_AMCL=1 CHECKPOINT=runs/predictive_checkpoints/v5-final.pt DURATION=30 bash scripts/run_predictive_async.sh evidence/2026-10-26/predictive-fault-camera > runs/predictive-fault-camera.log 2>&1
python3 scripts/analyze_sensor_fault.py evidence/2026-10-26/predictive-fault-camera --kind camera --start 4 --duration 2.5
