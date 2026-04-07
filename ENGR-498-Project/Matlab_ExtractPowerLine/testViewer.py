import sys
from PySide6.QtWidgets import QApplication

# Import your viewer
from testSemanticLidarViewer import SemanticViewer


def main():
    app = QApplication(sys.argv)

    viewer = SemanticViewer()
    viewer.initialize_viewer(sys.argv[1] if len(sys.argv) > 1 else None)
    viewer.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
