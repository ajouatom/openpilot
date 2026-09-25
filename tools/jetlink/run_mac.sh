#!/usr/bin/env bash
# Provision on the Mac itself. The comma never uploads or invents an ONNX model.
set -euo pipefail
[[ $(uname -s) == Darwin && $(uname -m) == arm64 ]] || { echo 'Apple Silicon Mac required'; exit 1; }
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
MODEL=${1:?path to the verified Cinque v2 ONNX required}
RUNTIME=${JETLINK_RUNTIME:-"$HOME/Library/Application Support/CarrotJetlink"}
python3 -m venv "$RUNTIME/venv"
"$RUNTIME/venv/bin/python" -m pip install 'numpy>=1.24,<3' 'onnx>=1.16' 'onnxruntime>=1.22' 'libusb1>=3.0'
export PYTHONPATH="$ROOT/third_party/jetlink"
"$RUNTIME/venv/bin/python" "$ROOT/tools/jetlink/verify_model.py" "$MODEL"
"$RUNTIME/venv/bin/python" "$ROOT/tools/jetlink/server.py" --backend ort --device coreml --cache "$RUNTIME/cache" --build "$MODEL"
exec "$RUNTIME/venv/bin/python" "$ROOT/tools/jetlink/server.py" --backend ort --device coreml --cache "$RUNTIME/cache" --transport usb
