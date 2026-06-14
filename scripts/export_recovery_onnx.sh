#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY_REPO="${PY_REPO:-$ROOT/../Serialleg_deploy_python}"
CHECKPOINT="${1:-${CHECKPOINT:-}}"
OUTPUT="${2:-${OUTPUT:-}}"

if [[ -z "${CHECKPOINT}" || -z "${OUTPUT}" ]]; then
    cat >&2 <<'EOF'
Usage: export_recovery_onnx.sh CHECKPOINT OUTPUT
Environment:
  PY_REPO   Path to Serialleg_deploy_python checkout
EOF
    exit 1
fi

if [[ ! -d "${PY_REPO}" ]]; then
    echo "[ERROR] Python source repo not found: ${PY_REPO}" >&2
    exit 1
fi

exec python3 - <<PY
from pathlib import Path
import sys
repo = Path(${PY_REPO@Q})
sys.path.insert(0, str(repo / "src"))
from se3_deploy.export_onnx import export_onnx
export_onnx(Path(${CHECKPOINT@Q}), Path(${OUTPUT@Q}))
PY
