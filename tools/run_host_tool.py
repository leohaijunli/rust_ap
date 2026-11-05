#!/usr/bin/env python3
"""Run a project tool with the Python 3.10 tomllib compatibility import."""

import runpy
import sys
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib
    sys.modules["tomllib"] = tomllib

if len(sys.argv) < 2:
    raise SystemExit("usage: run_host_tool.py TOOL.py [arguments]")
path = Path(sys.argv[1]).resolve()
if path.parent != Path(__file__).resolve().parent or path.suffix != ".py":
    raise SystemExit("tool must be a Python file in this project's tools directory")
sys.path.insert(0, str(path.parent))
sys.argv = [str(path), *sys.argv[2:]]
runpy.run_path(str(path), run_name="__main__")
