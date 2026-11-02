#!/usr/bin/env bash
# Install project-local Python tools and the pinned PX4 uploader.
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python="$root/.venv/bin/python"
if [[ ! -x "$python" ]]; then
    python3 -m venv "$root/.venv"
fi
"$python" -m pip install -r "$root/tools/requirements.txt"
"$python" - "$root/.venv/px4_uploader.py" <<'PY'
import hashlib
import sys
import tempfile
import urllib.request
from pathlib import Path

revision = "a2be9197b57b2d065e4c47ca7fc5b7565ee07282"
expected = "f05929e1035e693005677e088e4e7085503b2e8ab3d7c8785f7bd84cd48880f8"
destination = Path(sys.argv[1])
if destination.exists():
    data = destination.read_bytes()
else:
    url = f"https://raw.githubusercontent.com/PX4/PX4-Autopilot/{revision}/Tools/px4_uploader.py"
    with urllib.request.urlopen(url, timeout=30) as response:
        data = response.read(1024 * 1024)
if hashlib.sha256(data).hexdigest() != expected:
    sys.exit(f"PX4 uploader checksum mismatch: {destination}")
if not destination.exists():
    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        temporary = Path(directory) / destination.name
        temporary.write_bytes(data)
        temporary.replace(destination)
print(f"Pinned PX4 uploader: {destination} sha256={expected}")
PY
"$python" "$root/.venv/px4_uploader.py" --help >/dev/null
