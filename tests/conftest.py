"""Make source-tree imports deterministic for plain ``pytest`` and Docker builds."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PYTHON_PACKAGE_ROOT = ROOT / "drone_system"
for candidate in (PYTHON_PACKAGE_ROOT, ROOT):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))
