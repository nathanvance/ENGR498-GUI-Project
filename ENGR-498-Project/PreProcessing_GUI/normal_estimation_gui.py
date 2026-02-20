"""
GUI for LAS Point Cloud Normal Estimation

A simple PySide6 GUI for estimating normals using Open3D.
Can be used standalone or integrated into your main application.
"""

import numpy as np
import laspy
import open3d as o3d
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QSpinBox, QDoubleSpinBox, QGroupBox, QFileDialog, 
    QMessageBox, QProgressBar, QComboBox, QCheckBox, QApplication
)
from PySide6.QtCore import Qt, QThread, Signal
from pathlib import Path


class NormalEstimationThread(QThread):
    """Background thread for normal estimation"""
    progress = Signal(str)
    finished = Signal(str, np.ndarray)  # output_path, normals
    error = Signal(str)
    
    def __init__(self, input_path, radius, max_nn, orient_method):
        super().__init__()
        self.input_path = input_path
        self.radius = radius
        self.max_nn = max_nn
        self.orient_method = orient_method
        
    def run(self):
        try:
            # Load LAS
            self.progress.emit("Loading LAS file...")
            las = laspy.read(self.input_path)
            
            # Convert to Open3D
            self.progress.emit(f"Converting {len(las.points):,} points to Open3D format...")
            points = np.vstack((las.x, las.y, las.z)).T
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # Auto-compute radius if needed
            if self.radius is None:
                self.progress.emit("Computing optimal search radius...")
                pcd_tree = o3d.geometry.KDTreeFlann(pcd)
                distances = []
                num_samples = min(1000, len(pcd.points))
                indices = np.random.choice(len(pcd.points), num_samples, replace=False)
                
                for idx in indices:
                    [_, idx_nn, dist_nn] = pcd_tree.search_knn_vector_3d(pcd.points[idx], 2)
                    if len(dist_nn) > 1:
                        distances.append(np.sqrt(dist_nn[1]))
                
                self.radius = np.mean(distances) * 2.5
                self.progress.emit(f"Auto-computed radius: {self.radius:.4f}m")
            
            # Estimate normals
            self.progress.emit(f"Estimating normals (radius={self.radius:.4f}m, max_nn={self.max_nn})...")
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.radius,
                    max_nn=self.max_nn
                )
            )
            
            # Orient normals
            if self.orient_method == 'camera':
                self.progress.emit("Orienting normals towards camera...")
                pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
            elif self.orient_method == 'tangent':
                self.progress.emit("Orienting normals using tangent plane...")
                pcd.orient_normals_to_align_with_direction(
                    orientation_reference=np.array([0., 0., 1.])
                )
            
            # Extract normals
            normals = np.asarray(pcd.normals)
            
            # Generate output path
            input_path_obj = Path(self.input_path)
            output_path = str(input_path_obj.parent / f"{input_path_obj.stem}_with_normals{input_path_obj.suffix}")
            
            # Save with normals
            self.progress.emit("Saving LAS file with normals...")
            las_out = laspy.LasData(las.header)
            las_out.points = las.points
            
            las_out.add_extra_dim(laspy.ExtraBytesParams(name="NormalX", type=np.float32))
            las_out.add_extra_dim(laspy.ExtraBytesParams(name="NormalY", type=np.float32))
            las_out.add_extra_dim(laspy.ExtraBytesParams(name="NormalZ", type=np.float32))
            
            las_out.NormalX = normals[:, 0].astype(np.float32)
            las_out.NormalY = normals[:, 1].astype(np.float32)
            las_out.NormalZ = normals[:, 2].astype(np.float32)
            
            las_out.write(output_path)
            
            self.progress.emit("Complete!")
            self.finished.emit(output_path, normals)
            
        except Exception as e:
            self.error.emit(str(e))


class NormalEstimationGUI(QWidget):
    """GUI for estimating normals in LAS point clouds"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Point Cloud Normal Estimation")
        self.resize(600, 400)
        
        self.input_file = None
        self.thread = None
        
        self._setup_ui()
        
    def _setup_ui(self):
        """Setup the UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("LAS Point Cloud Normal Estimation")
        title.setStyleSheet("font-weight: bold; font-size: 16px;")
        layout.addWidget(title)
        
        # File selection
        file_group = QGroupBox("Input File")
        file_layout = QVBoxLayout()
        file_group.setLayout(file_layout)
        
        file_select_layout = QHBoxLayout()
        self.lbl_file = QLabel("No file selected")
        file_select_layout.addWidget(self.lbl_file)
        
        btn_browse = QPushButton("Browse...")
        btn_browse.clicked.connect(self.browse_file)
        file_select_layout.addWidget(btn_browse)
        
        file_layout.addLayout(file_select_layout)
        layout.addWidget(file_group)
        
        # Parameters
        params_group = QGroupBox("Normal Estimation Parameters")
        params_layout = QVBoxLayout()
        params_group.setLayout(params_layout)
        
        # Search radius
        radius_layout = QHBoxLayout()
        radius_layout.addWidget(QLabel("Search Radius (m):"))
        self.cb_auto_radius = QCheckBox("Auto-compute")
        self.cb_auto_radius.setChecked(True)
        self.cb_auto_radius.stateChanged.connect(self._toggle_radius)
        radius_layout.addWidget(self.cb_auto_radius)
        
        self.spin_radius = QDoubleSpinBox()
        self.spin_radius.setMinimum(0.01)
        self.spin_radius.setMaximum(10.0)
        self.spin_radius.setSingleStep(0.1)
        self.spin_radius.setValue(0.5)
        self.spin_radius.setEnabled(False)
        radius_layout.addWidget(self.spin_radius)
        params_layout.addLayout(radius_layout)
        
        # Max neighbors
        neighbors_layout = QHBoxLayout()
        neighbors_layout.addWidget(QLabel("Max Neighbors:"))
        self.spin_neighbors = QSpinBox()
        self.spin_neighbors.setMinimum(5)
        self.spin_neighbors.setMaximum(100)
        self.spin_neighbors.setValue(30)
        neighbors_layout.addWidget(self.spin_neighbors)
        neighbors_layout.addStretch()
        params_layout.addLayout(neighbors_layout)
        
        # Orientation method
        orient_layout = QHBoxLayout()
        orient_layout.addWidget(QLabel("Normal Orientation:"))
        self.combo_orient = QComboBox()
        self.combo_orient.addItems(["Camera (0,0,0)", "Tangent Plane", "None"])
        self.combo_orient.setCurrentIndex(0)
        orient_layout.addWidget(self.combo_orient)
        orient_layout.addStretch()
        params_layout.addLayout(orient_layout)
        
        layout.addWidget(params_group)
        
        # Info text
        info_text = QLabel(
            "ℹ️ Tips:\n"
            "• Auto-compute radius: Automatically determines optimal search distance\n"
            "• Max neighbors: More neighbors = smoother normals but slower computation\n"
            "• Camera orientation: Good for airborne LiDAR (ground-facing)\n"
            "• Tangent plane: Good for vertical surfaces"
        )
        info_text.setWordWrap(True)
        info_text.setStyleSheet("background-color: #f0f0f0; padding: 10px; border-radius: 5px;")
        layout.addWidget(info_text)
        
        # Progress
        self.progress_label = QLabel("")
        layout.addWidget(self.progress_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        self.btn_estimate = QPushButton("Estimate Normals")
        self.btn_estimate.setEnabled(False)
        self.btn_estimate.clicked.connect(self.estimate_normals)
        button_layout.addWidget(self.btn_estimate)
        
        button_layout.addStretch()
        layout.addLayout(button_layout)
        
        layout.addStretch()
        
    def _toggle_radius(self, state):
        """Toggle auto-compute radius"""
        self.spin_radius.setEnabled(state != Qt.Checked)
        
    def browse_file(self):
        """Browse for input LAS file"""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open LAS File",
            "",
            "LAS Files (*.las *.laz);;All Files (*)"
        )
        
        if filename:
            self.input_file = filename
            self.lbl_file.setText(Path(filename).name)
            self.btn_estimate.setEnabled(True)
            
    def estimate_normals(self):
        """Start normal estimation in background thread"""
        if not self.input_file:
            return
            
        # Get parameters
        if self.cb_auto_radius.isChecked():
            radius = None
        else:
            radius = self.spin_radius.value()
            
        max_nn = self.spin_neighbors.value()
        
        orient_idx = self.combo_orient.currentIndex()
        orient_map = {0: 'camera', 1: 'tangent', 2: None}
        orient_method = orient_map[orient_idx]
        
        # Disable UI during processing
        self.btn_estimate.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 0)  # Indeterminate
        
        # Start thread
        self.thread = NormalEstimationThread(
            self.input_file, radius, max_nn, orient_method
        )
        self.thread.progress.connect(self._update_progress)
        self.thread.finished.connect(self._on_complete)
        self.thread.error.connect(self._on_error)
        self.thread.start()
        
    def _update_progress(self, message):
        """Update progress label"""
        self.progress_label.setText(message)
        
    def _on_complete(self, output_path, normals):
        """Handle completion"""
        self.progress_bar.setVisible(False)
        self.btn_estimate.setEnabled(True)
        self.progress_label.setText("")
        
        QMessageBox.information(
            self,
            "Success",
            f"Normal estimation complete!\n\n"
            f"Output file: {output_path}\n"
            f"Estimated {len(normals):,} normals"
        )
        
    def _on_error(self, error_msg):
        """Handle error"""
        self.progress_bar.setVisible(False)
        self.btn_estimate.setEnabled(True)
        self.progress_label.setText("")
        
        QMessageBox.critical(
            self,
            "Error",
            f"Normal estimation failed:\n{error_msg}"
        )


if __name__ == "__main__":
    import sys
    
    app = QApplication(sys.argv)
    window = NormalEstimationGUI()
    window.show()
    sys.exit(app.exec())
