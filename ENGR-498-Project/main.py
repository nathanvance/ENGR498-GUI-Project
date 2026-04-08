import os
import subprocess
import sys


def _has_nvidia_gpu() -> bool:
    try:
        completed = subprocess.run(
            ["nvidia-smi", "-L"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return completed.returncode == 0
    except Exception:
        return False


if sys.platform.startswith("win") and not _has_nvidia_gpu():
    os.environ.setdefault("QT_OPENGL", "software")
    os.environ.setdefault("QSG_RHI_BACKEND", "software")

from PySide6.QtWidgets import QApplication

from testDashboard import DashboardTestWindow


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DashboardTestWindow()
    window.show()
    sys.exit(app.exec())
