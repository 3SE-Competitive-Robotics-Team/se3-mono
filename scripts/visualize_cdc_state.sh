#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
args=(--local-cdc)
if [[ -n "${CDC_VIS_HOST:-}" ]]; then
    args+=(--host "${CDC_VIS_HOST}")
else
    args+=(--host 0.0.0.0)
fi
if [[ -n "${CDC_VIS_PORT:-}" ]]; then
    args+=(--viewer-port "${CDC_VIS_PORT}")
fi
exec cargo run -p visualize_cdc_state -- "${args[@]}" "$@"
