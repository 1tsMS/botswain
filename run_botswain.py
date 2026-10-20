"""Start Botswain without VS Code: `python run_botswain.py` (or double-click Botswain.bat).

Uses the project's .venv even when started with another Python, so a double-click on this file works too.
Arguments are passed on (e.g. --robot configs/robots/fake.yaml).
"""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
VENV_PYTHON = ROOT / ".venv" / "Scripts" / "python.exe"


def main() -> int:
    if VENV_PYTHON.exists() and Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        return subprocess.call([str(VENV_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]], cwd=ROOT)
    sys.path.insert(0, str(ROOT))
    from botswain.__main__ import main as botswain_main

    return botswain_main(sys.argv[1:])


if __name__ == "__main__":
    sys.exit(main())
