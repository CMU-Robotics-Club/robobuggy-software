"""Allow pure tests to run before ROS message generation or source overlays."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
