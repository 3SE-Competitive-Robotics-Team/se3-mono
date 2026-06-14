#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

export SE3_TELEMETRY_LOG="${SE3_TELEMETRY_LOG:-off}"
export SE3_TELEMETRY_LOG_EVERY="${SE3_TELEMETRY_LOG_EVERY:-1}"
export SE3_TELEMETRY_FLUSH_EVERY="${SE3_TELEMETRY_FLUSH_EVERY:-25}"

has_checkpoint_arg=0
for arg in "$@"; do
  if [[ "${arg}" == "--checkpoint" ]]; then
    has_checkpoint_arg=1
    break
  fi
done

if [[ -z "${SE3_RECOVERY_CHECKPOINT:-}" && "${has_checkpoint_arg}" -eq 0 ]]; then
  echo "[ERROR] Set SE3_RECOVERY_CHECKPOINT or pass --checkpoint explicitly." >&2
  exit 1
fi

args=()
if [[ -n "${SE3_RECOVERY_CHECKPOINT:-}" && "${has_checkpoint_arg}" -eq 0 ]]; then
  args+=(--checkpoint "${SE3_RECOVERY_CHECKPOINT}")
fi
args+=(--ort-ep "${SE3_ORT_EP:-auto}" --port "${SE3_CDC_PORT:-auto}" --rate-hz 50)

exec cargo run -p locomotion -- "${args[@]}" "$@"
