#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export QT_QPA_PLATFORM=xcb
exec .venv-perception-live/bin/python perception/scripts/realsense_live.py "$@"
