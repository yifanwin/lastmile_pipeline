#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(dirname "$WORKSPACE")"
source "$ROOT/molmospaces/setup_env.sh"
export PYTHONPATH="$WORKSPACE/src:$PYTHONPATH"
export MUJOCO_GL=disable
export PYTHONDONTWRITEBYTECODE=1
PYTHON="$ROOT/molmospaces/.venv/bin/python"
cd "$WORKSPACE"
"$PYTHON" -m unittest discover -s tests -v
"$PYTHON" scripts/audit_assets.py
"$PYTHON" scripts/calibrate_model.py
