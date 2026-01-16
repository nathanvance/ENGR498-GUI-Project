import numpy as np
import laspy
import open3d as o3d
from PySide6.QtWidgets import QWidget, QVBoxLayout
from open3d.visualization import gui, rendering

class LidarViewerWidget(QWidget):
    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()
        self.setLayout(layout)

        # Create SceneWidget
        self.widget = gui.SceneWidget()

        # This works for your Open3D 0.19 Windows CPU build
        renderer = self.widget.renderer
        self.widget.scene = rendering.Open3DScene(renderer)

        layout.addWidget(self.widget)

        self.points_by_class = {}
        self.class_colors = {}

    def load_las(self, las_path):
        las = laspy.read(las_path)

        xyz = np.vstack([las.x, las.y, las.z]).T
        classifications = np.array(las.classification)

        unique_classes = np.unique(classifications)

        # Assign random colors
        for cls in unique_classes:
            self.class_colors[int(cls)] = np.random.rand(3)

        # Clear scene
        self.widget.scene.clear_geometry()

        classes_info = []

        for cls in unique_classes:
            mask = classifications == cls
            pts = xyz[mask]

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            color = np.tile(self.class_colors[int(cls)], (pts.shape[0], 1))
            pcd.colors = o3d.utility.Vector3dVector(color)

            self.points_by_class[int(cls)] = pcd

            # Add to viewer
            self.widget.scene.add_geometry(
                f"class_{cls}",
                pcd,
                rendering.MaterialRecord()
            )

            classes_info.append({
                "value": int(cls),
                "name": f"Class {cls}"  # You can replace with real names
            })

        bounds = self.widget.scene.bounding_box
        self.widget.setup_camera(60, bounds, bounds.get_center())

        return classes_info

    def toggle_class_visibility(self, cls_value, visible):
        if visible:
            pcd = self.points_by_class[cls_value]
            self.widget.scene.add_geometry(
                f"class_{cls_value}",
                pcd,
                rendering.MaterialRecord()
            )
        else:
            self.widget.scene.remove_geometry(f"class_{cls_value}")
