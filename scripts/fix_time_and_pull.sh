#!/usr/bin/env bash
set -euo pipefail

PROXY_URL="${PROXY_URL:-http://127.0.0.1:7897}"
TARGET_URL="${TARGET_URL:-https://www.baidu.com}"
TIME_URLS="${TIME_URLS:-http://www.baidu.com https://www.baidu.com http://connectivitycheck.gstatic.com/generate_204}"
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SUDO_PASS="${SUDO_PASS:-}"
REMOTE_NAME="${REMOTE_NAME:-origin}"
BRANCH_NAME="${BRANCH_NAME:-}"
KEEP_NTP_OFF="${KEEP_NTP_OFF:-0}"
DRY_RUN="${DRY_RUN:-0}"
TIME_ONLY="${TIME_ONLY:-0}"
SYNC_HWCLOCK="${SYNC_HWCLOCK:-1}"

usage() {
    cat <<'EOF'
Usage: fix_time_and_pull.sh [options]

Options:
  --time-only, --no-pull  Only sync system time, skip git pull
  --pull                  Enable git pull (default)
  -h, --help              Show help

Environment:
  PROXY_URL, TARGET_URL, TIME_URLS, REPO_DIR, SUDO_PASS
  REMOTE_NAME, BRANCH_NAME, KEEP_NTP_OFF, DRY_RUN, TIME_ONLY, SYNC_HWCLOCK
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --time-only|--no-pull)
            TIME_ONLY=1
            shift
            ;;
        --pull)
            TIME_ONLY=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

sudo_run() {
    if [[ "${DRY_RUN}" == "1" ]]; then
        echo "[DRY-RUN] sudo $*"
        return 0
    fi

    if [[ -n "${SUDO_PASS}" ]]; then
        printf '%s\n' "${SUDO_PASS}" | sudo -S "$@"
    else
        sudo "$@"
    fi
}

abs_int() {
    local n="$1"
    if (( n < 0 )); then
        echo $(( -n ))
    else
        echo "${n}"
    fi
}

if ! command -v curl >/dev/null 2>&1; then
    echo "[ERROR] curl is required but not found."
    exit 1
fi

if ! command -v date >/dev/null 2>&1; then
    echo "[ERROR] date is required but not found."
    exit 1
fi

if [[ "${TIME_ONLY}" != "1" ]] && ! command -v git >/dev/null 2>&1; then
    echo "[ERROR] git is required but not found."
    exit 1
fi

fetch_date_header() {
    local url="$1"
    local insecure="${2:-0}"
    local curl_args=(-sI --max-time 15 --connect-timeout 5)

    if [[ -n "${PROXY_URL}" ]]; then
        curl_args+=(--proxy "${PROXY_URL}")
    fi
    if [[ "${insecure}" == "1" ]]; then
        curl_args+=(-k)
    fi

    curl "${curl_args[@]}" "${url}" \
        | awk 'BEGIN{IGNORECASE=1} /^Date:/{sub(/\r$/, "", $0); print substr($0, 7); exit}'
}

DATE_HEADER=""
DATE_SOURCE=""
echo "[INFO] Fetching server time via proxy ${PROXY_URL} ..."

for url in ${TIME_URLS}; do
    echo "[INFO] Trying time source: ${url}"
    DATE_HEADER="$(fetch_date_header "${url}" 0 2>/dev/null || true)"

    if [[ -z "${DATE_HEADER}" && "${url}" == https://* ]]; then
        # On a 1970 clock, TLS verification often fails. Use this only as fallback.
        DATE_HEADER="$(fetch_date_header "${url}" 1 2>/dev/null || true)"
    fi

    if [[ -n "${DATE_HEADER}" ]]; then
        DATE_SOURCE="${url}"
        break
    fi
done

if [[ -z "${DATE_HEADER}" && -n "${TARGET_URL}" ]]; then
    echo "[INFO] Fallback trying TARGET_URL: ${TARGET_URL}"
    DATE_HEADER="$(fetch_date_header "${TARGET_URL}" 0 2>/dev/null || true)"
    if [[ -z "${DATE_HEADER}" && "${TARGET_URL}" == https://* ]]; then
        DATE_HEADER="$(fetch_date_header "${TARGET_URL}" 1 2>/dev/null || true)"
    fi
    if [[ -n "${DATE_HEADER}" ]]; then
        DATE_SOURCE="${TARGET_URL}"
    fi
fi

if [[ -z "${DATE_HEADER}" ]]; then
    echo "[ERROR] Failed to read Date header from all time sources through proxy."
    exit 1
fi

NEW_UTC="$(date -u -d "${DATE_HEADER}" "+%Y-%m-%d %H:%M:%S" 2>/dev/null || true)"
if [[ -z "${NEW_UTC}" ]]; then
    echo "[ERROR] Failed to parse Date header: ${DATE_HEADER}"
    exit 1
fi

echo "[INFO] Time source selected: ${DATE_SOURCE}"

WAS_NTP_ON=0
CAN_MANAGE_NTP=0
if command -v timedatectl >/dev/null 2>&1; then
    can_ntp="$(timedatectl show -p CanNTP --value 2>/dev/null || true)"
    ntp_enabled="$(timedatectl show -p NTP --value 2>/dev/null || true)"

    if [[ "${can_ntp}" != "no" ]]; then
        CAN_MANAGE_NTP=1
    fi
    if [[ "${ntp_enabled}" == "yes" ]]; then
        WAS_NTP_ON=1
    fi

    if [[ "${CAN_MANAGE_NTP}" -eq 1 && "${WAS_NTP_ON}" -eq 1 ]]; then
        echo "[INFO] Auto time is enabled; temporarily disabling NTP ..."
        sudo_run timedatectl set-ntp false
        sleep 1
    fi
fi

echo "[INFO] Setting system UTC time to: ${NEW_UTC}"
if [[ "${DRY_RUN}" == "1" ]]; then
    echo "[DRY-RUN] Skip setting system time"
else
    if ! sudo_run date -u -s "${NEW_UTC}" >/dev/null 2>&1; then
        if command -v timedatectl >/dev/null 2>&1; then
            sudo_run timedatectl set-time "${NEW_UTC} UTC"
        else
            echo "[ERROR] Failed to set system time with date command."
            exit 1
        fi
    fi
fi

if [[ "${DRY_RUN}" != "1" ]]; then
    target_epoch="$(date -u -d "${NEW_UTC}" +%s)"
    current_epoch="$(date -u +%s)"
    drift="$(abs_int $(( current_epoch - target_epoch )))"
    if (( drift > 10 )); then
        echo "[WARN] Time drift after set is ${drift}s; auto time may have overridden the manual set."
    fi

    if [[ "${SYNC_HWCLOCK}" == "1" ]] && command -v hwclock >/dev/null 2>&1; then
        if ! sudo_run hwclock --systohc >/dev/null 2>&1; then
            echo "[WARN] Failed to sync system time to RTC with hwclock."
        fi
    fi
fi

if [[ "${CAN_MANAGE_NTP}" -eq 1 && "${WAS_NTP_ON}" -eq 1 && "${KEEP_NTP_OFF}" != "1" ]]; then
    echo "[INFO] Restoring NTP auto time ..."
    sudo_run timedatectl set-ntp true
fi

echo "[INFO] Local time after sync: $(date "+%Y-%m-%d %H:%M:%S %Z")"

if [[ "${TIME_ONLY}" == "1" ]]; then
    echo "[OK] Time sync completed (pull skipped by --time-only/--no-pull)."
    exit 0
fi

if [[ -z "${BRANCH_NAME}" ]]; then
    BRANCH_NAME="$(git -C "${REPO_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo main)"
fi

echo "[INFO] Pulling latest changes in ${REPO_DIR} (${REMOTE_NAME}/${BRANCH_NAME}) ..."
if [[ "${DRY_RUN}" == "1" ]]; then
    echo "[DRY-RUN] git -C ${REPO_DIR} -c http.proxy=${PROXY_URL} pull --rebase --autostash ${REMOTE_NAME} ${BRANCH_NAME}"
else
    git -C "${REPO_DIR}" -c http.proxy="${PROXY_URL}" pull --rebase --autostash "${REMOTE_NAME}" "${BRANCH_NAME}"
fi

echo "[OK] Time sync and repository update completed."