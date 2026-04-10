# views/semantic_viewer.py
from pathlib import Path
import numpy as np
import laspy
import pyvista as pv

from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel,
    QCheckBox, QScrollArea, QSizePolicy
)
from PySide6.QtCore import Qt, Signal
from pyvistaqt import QtInteractor

from project_paths import DEFAULT_LAS_PATH
from theme import THEME, button_style

LAS_PATH = DEFAULT_LAS_PATH

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
    try:
        classes = las.classification
    except Exception:
        classes = np.zeros(xyz.shape[0], dtype=int)
    return xyz, classes


def generate_deterministic_colors(n, seed=0):
    rng = np.random.default_rng(seed)
    colors = [tuple(int(x) for x in rng.integers(30, 230, size=3)) for _ in range(n)]
    return colors


class SemanticViewer(QWidget):
    backRequested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("semanticViewer")
        layout = QHBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        self.setLayout(layout)

        # Left side: 3D viewer
        self.plotter = QtInteractor(self)
        self.plotter.setMinimumSize(900, 700)
        layout.addWidget(self.plotter, stretch=3)

        # Right side: controls
        side_panel_host = QWidget()
        side_panel_host.setProperty("card", True)
        side_panel_host.setStyleSheet(
            f"QWidget {{ background-color: {THEME['pane']}; border: 1px solid {THEME['border']}; border-radius: {THEME['radius_md']}; }}"
        )
        side_panel = QVBoxLayout()
        side_panel.setContentsMargins(12, 12, 12, 12)
        side_panel.setSpacing(10)
        side_panel_host.setLayout(side_panel)
        layout.addWidget(side_panel_host, stretch=1)

        title = QLabel("Semantic Classes")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {THEME['text']};")
        side_panel.addWidget(title)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QScrollArea.NoFrame)
        scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout()
        scroll_content.setLayout(self.scroll_layout)
        self.scroll.setWidget(scroll_content)
        side_panel.addWidget(self.scroll)

        self.back_button = QPushButton("Back")
        self.back_button.setStyleSheet(button_style(THEME["accent"], THEME["accent_hover"]))
        self.back_button.clicked.connect(self.backRequested.emit)
        side_panel.addWidget(self.back_button)

        # Storage
        self.xyz = None
        self.classes = None
        self.unique_classes = None
        self.class_actors = {}
        self.class_colors = {}
        self.class_checkboxes = {}
        self.las_actor = None

    def load_las_file(self, path=LAS_PATH):
        path = Path(path)
        self.xyz, self.classes = load_las_xyz_and_classes(path)
        self.unique_classes = np.unique(self.classes)
        self._render_class_list()
        self._render_las_cloud()

    def _render_class_list(self):
        # Clear old
        for i in reversed(range(self.scroll_layout.count())):
            widget = self.scroll_layout.itemAt(i).widget()
            if widget:
                widget.setParent(None)

        self.class_checkboxes = {}
        colors = generate_deterministic_colors(len(self.unique_classes), seed=42)
        self.class_colors = {}

        for idx, cls_id in enumerate(self.unique_classes):
            name = CLASS_NAME_MAP.get(int(cls_id), f"Class {cls_id}")
            color = colors[idx]
            self.class_colors[int(cls_id)] = color

            row = QHBoxLayout()
            cb = QCheckBox(f"{cls_id} - {name}")
            cb.setChecked(True)
            cb.stateChanged.connect(lambda state, c=int(cls_id): self._toggle_class(c, state))
            self.class_checkboxes[int(cls_id)] = cb
            row.addWidget(cb)

            swatch = QLabel()
            swatch.setFixedSize(16, 16)
            swatch.setStyleSheet(
                f"background-color: rgb({color[0]}, {color[1]}, {color[2]}); border: 1px solid {THEME['border_strong']}; border-radius: 4px;"
            )
            row.addWidget(swatch)

            container = QWidget()
            container.setLayout(row)
            self.scroll_layout.addWidget(container)

        self.scroll_layout.addStretch()

    def _render_las_cloud(self):
        self.plotter.clear()
        self.class_actors = {}

        # Add dim gray all-cloud background for context
        pdata_all = pv.PolyData(self.xyz)
        self.las_actor = self.plotter.add_mesh(
            pdata_all,
            color=(160, 160, 160),
            point_size=2,
            render_points_as_spheres=True,
            name="las_background"
        )
        try:
            self.las_actor.SetVisibility(1)
        except Exception:
            pass

        for cls_id in self.unique_classes:
            mask = self.classes == cls_id
            pts = self.xyz[mask]
            if len(pts) == 0:
                continue

            pdata = pv.PolyData(pts)
            actor = self.plotter.add_mesh(
                pdata,
                color=self.class_colors[int(cls_id)],
                point_size=4,
                render_points_as_spheres=True,
                name=f"class_{int(cls_id)}"
            )
            self.class_actors[int(cls_id)] = actor

        self.plotter.reset_camera()
        self.plotter.render()

    def _toggle_class(self, cls_id, state):
        actor = self.class_actors.get(cls_id)
        if actor is not None:
            actor.SetVisibility(state == Qt.Checked)
            self.plotter.render()
