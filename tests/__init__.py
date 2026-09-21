import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "src" / "core"))
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
sys.path.insert(0, str(REPO_ROOT / "src" / "analysis"))
sys.path.insert(0, str(REPO_ROOT / "figures"))
