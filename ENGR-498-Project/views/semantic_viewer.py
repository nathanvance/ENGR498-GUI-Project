# views/semantic_viewer.py
import numpy as np
import laspy
import pyvista as pv

from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel,
    QCheckBox, QScrollArea, QSizePolicy
)
from PySide6.QtCore import Qt, Signal
from pyvistaqt import QtInteractor

# Hardcoded path per Option 2 — update to your actual LAS path
LAS_PATH = r"C:\Users\henry\Downloads\ENGR-498-Project\assets\testSemanticPC.las"

# Map class ID -> human readable name (optional)
CLASS_NAME_MAP = {
    0: "Created, never classified",
    1: "Unclassified",
    2: "Ground",
    3: "Low Vegetation",
    5: "High Vegetation"
}

def load_las_xyz_and_classes(path):
    las = laspy.read(path)
    xyz = np.vstack((las.x, las.y, las.z)).T
    classes = las.classification
    return xyz, classes

def generate_deterministic_colors(unique_classes, seed=0):
    rng = np.random.default_rng(seed)
    colors = {}
    for c in unique_classes:
        # PyVista expects 0-255 ints for color arguments
        colors[c] = tuple(int(x) for x in rng.integers(30, 230, size=3))
    return colors

class SemanticViewer(QWidget):
    # Signal to ask MainWindow to go back to dashboard
    backRequested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI layout
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # Left: PyVista QtInteractor
        self.plotter = QtInteractor(self)
        self.plotter.setMinimumSize(800, 600)
        main_layout.addWidget(self.plotter, stretch=3)

        # Right: sidebar for checkboxes + back button
        sidebar = QWidget()
        sidebar_layout = QVBoxLayout()
        sidebar.setLayout(sidebar_layout)

        # Top row: Back button
        top_row = QWidget()
        top_row_layout = QHBoxLayout()
        top_row_layout.setContentsMargins(0, 0, 0, 0)
        top_row.setLayout(top_row_layout)
        back_btn = QPushButton("← Back")
        back_btn.clicked.connect(self._on_back)
        top_row_layout.addWidget(back_btn)
        sidebar_layout.addWidget(top_row)

        title = QLabel("Semantic Classes")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        sidebar_layout.addWidget(title)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout()
        scroll_content.setLayout(scroll_layout)
        scroll.setWidget(scroll_content)
        self.scroll_layout = scroll_layout

        sidebar_layout.addWidget(scroll)
        sidebar_layout.addStretch()
        main_layout.addWidget(sidebar, stretch=1)

        # Storage
        self.actors = {}     # class -> actor
        self.checkboxes = {}
        self.class_colors = {}

        # Data placeholders
        self.xyz = None
        self.classes = None
        self.unique_classes = None

    def _on_back(self):
        # optional: clear the scene to free memory if you want
        try:
            self.plotter.clear()
        except Exception:
            pass
        self.backRequested.emit()

    def load_las_file(self, path=LAS_PATH):
        # Load LAS and render (path defaults to hardcoded LAS_PATH)
        self.xyz, self.classes = load_las_xyz_and_classes(path)
        unique = np.unique(self.classes)
        # Optionally filter to only known classes in CLASS_NAME_MAP
        # If you want all classes, comment out the next line
        self.unique_classes = np.array([c for c in unique if c in CLASS_NAME_MAP])
        if self.unique_classes.size == 0:
            # fallback to all classes if none match the map
            self.unique_classes = unique

        # deterministic colors
        self.class_colors = generate_deterministic_colors(self.unique_classes, seed=42)

        # Build the scene
        self._render_scene()

    def _render_scene(self):
        # Remove any existing actors
        self.plotter.clear()
        self.actors.clear()
        # Clear sidebar layout (remove old widgets)
        for i in reversed(range(self.scroll_layout.count())):
            w = self.scroll_layout.itemAt(i).widget()
            if w is not None:
                w.setParent(None)

        # Add one actor per class (colored point cloud)
        for c in self.unique_classes:
            mask = (self.classes == c)
            pts = self.xyz[mask]
            if pts.shape[0] == 0:
                continue

            pdata = pv.PolyData(pts)
            color = self.class_colors[int(c)]

            # Add mesh/points; return value is a vtk actor or wrapper
            actor = self.plotter.add_mesh(
                pdata,
                color=color,
                point_size=3,
                render_points_as_spheres=True,
                name=f"class_{int(c)}"
            )
            self.actors[int(c)] = actor

            # Create checkbox row with color square
            name = CLASS_NAME_MAP.get(int(c), f"Class {int(c)}")
            cb = QCheckBox(f"{int(c)} — {name}")
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_checkbox_changed)
            self.checkboxes[int(c)] = cb

            # color square QLabel
            color_hex = '#{:02x}{:02x}{:02x}'.format(*color)
            from PySide6.QtWidgets import QWidget, QLabel, QHBoxLayout
            row = QWidget()
            row_layout = QHBoxLayout()
            row_layout.setContentsMargins(0, 0, 0, 0)
            color_square = QLabel()
            color_square.setFixedSize(18, 18)
            color_square.setStyleSheet(f"background-color: {color_hex}; border: 1px solid #333;")
            row_layout.addWidget(color_square)
            row_layout.addWidget(cb)
            row.setLayout(row_layout)
            self.scroll_layout.addWidget(row)

        self.plotter.reset_camera()
        self.plotter.render()

    def _on_checkbox_changed(self, state):
        # Toggle visibility for each actor according to checkbox state
        for c, cb in self.checkboxes.items():
            actor = self.actors.get(int(c))
            if actor is None:
                continue
            visible = cb.isChecked()
            try:
                # vtkActor API
                actor.SetVisibility(1 if visible else 0)
            except Exception:
                # pyvista wrapper fallback
                try:
                    actor.actor.SetVisibility(1 if visible else 0)
                except Exception:
                    pass
        self.plotter.render()
