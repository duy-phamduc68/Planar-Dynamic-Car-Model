"""Repository entry point for running the simulator from project root.

Usage:
    python main.py
"""

from pathlib import Path
import os
import sys


ROOT = Path(__file__).resolve().parent
SRC_DIR = ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


def _venv_python_path():
    candidates = [
        ROOT / ".venv" / "Scripts" / "python.exe",
        ROOT / ".venv" / "bin" / "python",
    ]
    for p in candidates:
        if p.exists():
            return p
    return None


def _ensure_project_python():
    vpy = _venv_python_path()
    if vpy is None:
        return
    current = Path(sys.executable).resolve()
    if current == vpy.resolve():
        return
    # Re-exec into project virtualenv so `python main.py` works from repo root.
    os.execv(str(vpy), [str(vpy), str(ROOT / "main.py"), *sys.argv[1:]])


_ensure_project_python()

try:
    from src.simulator import Simulator  # noqa: E402
except ModuleNotFoundError as exc:
    if exc.name in ("pygame", "pygame_ce"):
        raise SystemExit(
            "Missing dependency: pygame. Activate the project virtual environment or install requirements.\n"
            "Example: source .venv/Scripts/activate && python main.py"
        )
    raise


if __name__ == "__main__":
    Simulator().run()
