import sys

from PySide6.QtWidgets import QApplication

from testDashboard import DashboardTestWindow


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DashboardTestWindow()
    window.show()
    sys.exit(app.exec())
