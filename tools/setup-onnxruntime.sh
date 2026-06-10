#!/usr/bin/env sh
set -eu

VERSION="${SE3_ORT_VERSION:-1.24.2}"
BASE_DIR="${SE3_LIB_DIR:-/opt/se3/lib}"
if [ -n "${SE3_ORT_DIR:-}" ]; then
    INSTALL_DIR="$SE3_ORT_DIR"
    CURRENT_DIR="$SE3_ORT_DIR"
else
    INSTALL_DIR="${BASE_DIR}/onnxruntime-${VERSION}"
    CURRENT_DIR="${BASE_DIR}/onnxruntime"
fi
LIB_PATH="${CURRENT_DIR}/lib/libonnxruntime.so"
INSTALL_LIB_PATH="${INSTALL_DIR}/lib/libonnxruntime.so"
MODE="${1:-install}"

usage() {
    cat <<EOF
Usage:
  $0            Download ONNX Runtime if libonnxruntime.so is missing.
  $0 --check    Check libonnxruntime.so exists, without downloading.

Environment:
  SE3_LIB_DIR       SE3 private library directory. Default: /opt/se3/lib
  SE3_ORT_DIR       Exact install directory. Overrides SE3_LIB_DIR.
  SE3_ORT_VERSION   ONNX Runtime version. Default: 1.24.2
  SE3_ORT_URL       Override download URL for a platform-specific package.
EOF
}

download() {
    url="$1"
    output="$2"

    if command -v curl >/dev/null 2>&1; then
        curl -fL "$url" -o "$output"
    elif command -v wget >/dev/null 2>&1; then
        wget -O "$output" "$url"
    else
        echo "error: curl or wget is required to download ONNX Runtime" >&2
        exit 1
    fi
}

detect_default_url() {
    os="$(uname -s)"
    if [ "$os" != "Linux" ]; then
        echo "error: default ONNX Runtime download only supports Linux, got: $os" >&2
        echo "set SE3_ORT_URL to a platform-specific ONNX Runtime package" >&2
        exit 1
    fi

    arch="$(uname -m)"
    case "$arch" in
        x86_64|amd64)
            package_arch="x64"
            ;;
        aarch64|arm64)
            package_arch="aarch64"
            ;;
        *)
            echo "error: unsupported architecture: $arch" >&2
            echo "set SE3_ORT_URL to a platform-specific ONNX Runtime package" >&2
            exit 1
            ;;
    esac

    echo "https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/onnxruntime-linux-${package_arch}-${VERSION}.tgz"
}

check_library() {
    if [ -f "$LIB_PATH" ]; then
        echo "ONNX Runtime library found: $LIB_PATH"
        exit 0
    fi

    echo "ONNX Runtime library missing: $LIB_PATH" >&2
    echo "run tools/setup-onnxruntime.sh before starting se3 processes" >&2
    exit 1
}

link_current() {
    if [ "$CURRENT_DIR" = "$INSTALL_DIR" ]; then
        return
    fi

    if [ -e "$CURRENT_DIR" ] && [ ! -L "$CURRENT_DIR" ]; then
        echo "error: current ONNX Runtime path exists and is not a symlink: $CURRENT_DIR" >&2
        exit 1
    fi

    if [ -L "$CURRENT_DIR" ]; then
        rm -f "$CURRENT_DIR"
    fi

    ln -s "$(basename "$INSTALL_DIR")" "$CURRENT_DIR"
}

case "$MODE" in
    install)
        ;;
    --check|check)
        check_library
        ;;
    -h|--help|help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

if [ -f "$LIB_PATH" ]; then
    echo "ONNX Runtime library already exists: $LIB_PATH"
    exit 0
fi

if [ -f "$INSTALL_LIB_PATH" ]; then
    link_current
    echo "ONNX Runtime library already exists: $LIB_PATH"
    exit 0
fi

URL="${SE3_ORT_URL:-$(detect_default_url)}"
TMP_DIR="$(mktemp -d)"
ARCHIVE="${TMP_DIR}/onnxruntime.tgz"
EXTRACT_DIR="${TMP_DIR}/extract"

cleanup() {
    rm -rf "$TMP_DIR"
}
trap cleanup EXIT HUP INT TERM

mkdir -p "$INSTALL_DIR"
mkdir -p "$EXTRACT_DIR"

echo "Downloading ONNX Runtime from: $URL"
download "$URL" "$ARCHIVE"

tar -xzf "$ARCHIVE" -C "$EXTRACT_DIR"

PACKAGE_DIR="$(find "$EXTRACT_DIR" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
if [ -z "$PACKAGE_DIR" ]; then
    echo "error: downloaded archive did not contain an ONNX Runtime directory" >&2
    exit 1
fi

if [ ! -f "$PACKAGE_DIR/lib/libonnxruntime.so" ]; then
    echo "error: downloaded archive does not contain lib/libonnxruntime.so" >&2
    exit 1
fi

cp -R "$PACKAGE_DIR"/. "$INSTALL_DIR"/
link_current

if [ ! -f "$LIB_PATH" ]; then
    echo "error: install finished but libonnxruntime.so is still missing: $LIB_PATH" >&2
    exit 1
fi

echo "ONNX Runtime installed: $LIB_PATH"
echo "Set ORT_DYLIB_PATH=$LIB_PATH before starting se3 processes."
