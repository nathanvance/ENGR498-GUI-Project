import numpy as np
import laspy
import pyvista as pv
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel,
    QSlider, QSpinBox, QDoubleSpinBox, QGroupBox, QCheckBox,
    QSplitter, QFileDialog, QMessageBox
)
from PySide6.QtGui import QUndoStack, QUndoCommand
from PySide6.QtCore import Qt, Signal, QTimer
from pyvistaqt import QtInteractor
from scipy.spatial import cKDTree
import threading


class FilterCommand(QUndoCommand):
    """Undo command for point cloud filtering operations"""
    def __init__(self, viewer, old_data, new_data, operation_name):
        super().__init__(operation_name)
        self.viewer = viewer
        # old_data and new_data are tuples of (xyz, colors, intensity, normals)
        # The numpy arrays inside are already copied by the calling code
        self.old_data = old_data
        self.new_data = new_data
        
    def undo(self):
        self.viewer._apply_data(self.old_data)
        
    def redo(self):
        self.viewer._apply_data(self.new_data)


class PointCloudFilterViewer(QWidget):
    """
    Point cloud viewer with filtering, downsampling, and denoising capabilities.
    Features undo/redo functionality and preserves all point attributes.
    """
    
    def __init__(self, parent=None, filename=None):
        super().__init__(parent)
        self.setWindowTitle("Point Cloud Filter Viewer")
        self.resize(1400, 800)

        #pass in filename
        self.filename = filename

        # Data storage
        self.original_xyz = None
        self.original_colors = None
        self.original_intensity = None
        self.original_normals = None
        
        self.current_xyz = None
        self.current_colors = None
        self.current_intensity = None
        self.current_normals = None
        
        # Actor references
        self.point_actor = None
        
        # Undo stack
        self.undo_stack = QUndoStack(self)
        
        # Build UI
        self._setup_ui()
        
    def _setup_ui(self):
        """Setup the main UI layout"""
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)
        
        # Main splitter
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left: 3D Viewer
        viewer_widget = self._create_viewer_widget()
        splitter.addWidget(viewer_widget)
        
        # Right: Control Panel
        control_widget = self._create_control_panel()
        splitter.addWidget(control_widget)
        
        # Set initial sizes
        splitter.setSizes([900, 500])
        
    def _create_viewer_widget(self):
        """Create the 3D viewer widget"""
        widget = QWidget()
        layout = QVBoxLayout()
        widget.setLayout(layout)
        
        # PyVista plotter
        self.plotter = QtInteractor(self)
        self.plotter.setMinimumSize(800, 600)
        layout.addWidget(self.plotter)
        
        # Viewer controls
        controls_layout = QHBoxLayout()
        
        btn_load = QPushButton("Load LAS File")
        btn_load.clicked.connect(self.load_las_file)
        controls_layout.addWidget(btn_load)
        
        btn_reset = QPushButton("Reset to Original")
        btn_reset.clicked.connect(self.reset_to_original)
        controls_layout.addWidget(btn_reset)
        
        btn_undo = QPushButton("Undo")
        btn_undo.clicked.connect(self.undo_stack.undo)
        controls_layout.addWidget(btn_undo)
        
        btn_redo = QPushButton("Redo")
        btn_redo.clicked.connect(self.undo_stack.redo)
        controls_layout.addWidget(btn_redo)
        
        #we might need to pass in the file path into the filter gui to save the las file in the assets path for the correct scan
        btn_save = QPushButton("Save LAS and Return")
        btn_save.clicked.connect(self.save_las_file)
        controls_layout.addWidget(btn_save)
        
        controls_layout.addStretch()
        
        # Point count label
        self.lbl_point_count = QLabel("Points: 0")
        controls_layout.addWidget(self.lbl_point_count)
        
        layout.addLayout(controls_layout)
        
        return widget
        
    def _create_control_panel(self):
        """Create the control panel with filtering options"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        widget.setLayout(layout)
        
        title = QLabel("Filtering Controls")
        title.setStyleSheet("font-weight: bold; font-size: 16px;")
        layout.addWidget(title)
        
        layout.addSpacing(10)
        
        # Downsampling controls
        layout.addWidget(self._create_downsample_group())
        
        # Denoising controls
        layout.addWidget(self._create_denoise_group())
        
        # Height filter controls
        layout.addWidget(self._create_height_filter_group())
        
        layout.addStretch()
        
        return widget
        
    def _create_downsample_group(self):
        """Create downsampling control group"""
        group = QGroupBox("Downsampling")
        layout = QVBoxLayout()
        group.setLayout(layout)
        
        # Voxel size slider
        lbl = QLabel("Voxel Size (m):")
        layout.addWidget(lbl)
        
        slider_layout = QHBoxLayout()
        self.slider_voxel = QSlider(Qt.Horizontal)
        self.slider_voxel.setMinimum(1)
        self.slider_voxel.setMaximum(200)  # 0.01 to 2.0 meters
        self.slider_voxel.setValue(10)  # 0.1m default
        slider_layout.addWidget(self.slider_voxel)
        
        self.spin_voxel = QDoubleSpinBox()
        self.spin_voxel.setMinimum(0.01)
        self.spin_voxel.setMaximum(2.0)
        self.spin_voxel.setSingleStep(0.01)
        self.spin_voxel.setValue(0.1)
        # Connect slider and spinbox bidirectionally
        self.slider_voxel.valueChanged.connect(lambda v: self.spin_voxel.setValue(v / 100.0))
        self.spin_voxel.valueChanged.connect(lambda v: self.slider_voxel.setValue(int(v * 100)))
        slider_layout.addWidget(self.spin_voxel)
        
        layout.addLayout(slider_layout)
        
        # Apply button
        btn_apply = QPushButton("Apply Downsampling")
        btn_apply.clicked.connect(self.apply_downsampling)
        layout.addWidget(btn_apply)
        
        return group
        
    def _create_denoise_group(self):
        """Create denoising control group"""
        group = QGroupBox("Statistical Outlier Removal (Denoising)")
        layout = QVBoxLayout()
        group.setLayout(layout)
        
        # Number of neighbors
        lbl1 = QLabel("Number of Neighbors:")
        layout.addWidget(lbl1)
        
        slider_layout1 = QHBoxLayout()
        self.slider_neighbors = QSlider(Qt.Horizontal)
        self.slider_neighbors.setMinimum(5)
        self.slider_neighbors.setMaximum(100)
        self.slider_neighbors.setValue(20)
        slider_layout1.addWidget(self.slider_neighbors)
        
        self.spin_neighbors = QSpinBox()
        self.spin_neighbors.setMinimum(5)
        self.spin_neighbors.setMaximum(100)
        self.spin_neighbors.setValue(20)
        # Connect slider and spinbox bidirectionally
        self.slider_neighbors.valueChanged.connect(self.spin_neighbors.setValue)
        self.spin_neighbors.valueChanged.connect(self.slider_neighbors.setValue)
        slider_layout1.addWidget(self.spin_neighbors)
        
        layout.addLayout(slider_layout1)
        
        # Standard deviation multiplier
        lbl2 = QLabel("Std Dev Multiplier:")
        layout.addWidget(lbl2)
        
        slider_layout2 = QHBoxLayout()
        self.slider_std = QSlider(Qt.Horizontal)
        self.slider_std.setMinimum(10)
        self.slider_std.setMaximum(50)
        self.slider_std.setValue(20)  # 2.0 default
        slider_layout2.addWidget(self.slider_std)
        
        self.spin_std = QDoubleSpinBox()
        self.spin_std.setMinimum(1.0)
        self.spin_std.setMaximum(5.0)
        self.spin_std.setSingleStep(0.1)
        self.spin_std.setValue(2.0)
        # Connect slider and spinbox bidirectionally
        self.slider_std.valueChanged.connect(lambda v: self.spin_std.setValue(v / 10.0))
        self.spin_std.valueChanged.connect(lambda v: self.slider_std.setValue(int(v * 10)))
        slider_layout2.addWidget(self.spin_std)
        
        layout.addLayout(slider_layout2)
        
        # Apply button
        btn_apply = QPushButton("Apply Denoising")
        btn_apply.clicked.connect(self.apply_denoising)
        layout.addWidget(btn_apply)
        
        return group
        
    def _create_height_filter_group(self):
        """Create height filter control group"""
        group = QGroupBox("Height Filter")
        layout = QVBoxLayout()
        group.setLayout(layout)
        
        # Min height
        lbl1 = QLabel("Min Height (m):")
        layout.addWidget(lbl1)
        
        slider_layout1 = QHBoxLayout()
        self.slider_min_height = QSlider(Qt.Horizontal)
        self.slider_min_height.setMinimum(-100)
        self.slider_min_height.setMaximum(100)
        self.slider_min_height.setValue(-100)
        slider_layout1.addWidget(self.slider_min_height)
        
        self.spin_min_height = QDoubleSpinBox()
        self.spin_min_height.setMinimum(-1000)
        self.spin_min_height.setMaximum(1000)
        self.spin_min_height.setValue(-1000)
        # Connect slider and spinbox bidirectionally
        self.slider_min_height.valueChanged.connect(lambda v: self.spin_min_height.setValue(v * 10.0))
        self.spin_min_height.valueChanged.connect(lambda v: self.slider_min_height.setValue(int(v / 10)))
        slider_layout1.addWidget(self.spin_min_height)
        
        layout.addLayout(slider_layout1)
        
        # Max height
        lbl2 = QLabel("Max Height (m):")
        layout.addWidget(lbl2)
        
        slider_layout2 = QHBoxLayout()
        self.slider_max_height = QSlider(Qt.Horizontal)
        self.slider_max_height.setMinimum(-100)
        self.slider_max_height.setMaximum(100)
        self.slider_max_height.setValue(100)
        slider_layout2.addWidget(self.slider_max_height)
        
        self.spin_max_height = QDoubleSpinBox()
        self.spin_max_height.setMinimum(-1000)
        self.spin_max_height.setMaximum(1000)
        self.spin_max_height.setValue(1000)
        # Connect slider and spinbox bidirectionally
        self.slider_max_height.valueChanged.connect(lambda v: self.spin_max_height.setValue(v * 10.0))
        self.spin_max_height.valueChanged.connect(lambda v: self.slider_max_height.setValue(int(v / 10)))
        slider_layout2.addWidget(self.spin_max_height)
        
        layout.addLayout(slider_layout2)
        
        # Apply button
        btn_apply = QPushButton("Apply Height Filter")
        btn_apply.clicked.connect(self.apply_height_filter)
        layout.addWidget(btn_apply)
        
        return group
        
    # ========================
    # File operations
    # ========================
    
    def load_las_file(self):
        print("Load LAS file clicked")
        if self.filename is not None:
            return

        else:
            """Load a LAS file"""
            filename, _ = QFileDialog.getOpenFileName(
                self,
                "Open LAS File",
                "",
                "LAS Files (*.las *.laz);;All Files (*)"
            )
            
            if not filename:
                return
        print(f"Loading LAS file: {self.filename}")

        try:
            las = laspy.read(filename)
            
            # Extract coordinates
            self.original_xyz = np.vstack((las.x, las.y, las.z)).T
            
            # Extract colors if available
            if hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue'):
                colors = np.vstack((las.red, las.green, las.blue)).T
                # Normalize to 0-255 if needed
                if colors.max() > 255:
                    colors = (colors / 65535 * 255).astype(np.uint8)
                self.original_colors = colors
            else:
                self.original_colors = None
            
            # Extract intensity if available
            if hasattr(las, 'intensity'):
                self.original_intensity = np.array(las.intensity)
            else:
                self.original_intensity = None
                
            # Extract normals if available (NormalX, NormalY, NormalZ extra bytes)
            if hasattr(las, 'NormalX') and hasattr(las, 'NormalY') and hasattr(las, 'NormalZ'):
                self.original_normals = np.vstack((las.NormalX, las.NormalY, las.NormalZ)).T
            else:
                self.original_normals = None
                
            # Set as current data
            self.current_xyz = self.original_xyz.copy()
            self.current_colors = self.original_colors.copy() if self.original_colors is not None else None
            self.current_intensity = self.original_intensity.copy() if self.original_intensity is not None else None
            self.current_normals = self.original_normals.copy() if self.original_normals is not None else None
            
            # Clear undo stack
            self.undo_stack.clear()
            
            # Update height filter ranges based on data
            z_min, z_max = self.original_xyz[:, 2].min(), self.original_xyz[:, 2].max()
            self.spin_min_height.setValue(z_min)
            self.spin_max_height.setValue(z_max)
            self.slider_min_height.setValue(int(z_min / 10))
            self.slider_max_height.setValue(int(z_max / 10))
            
            # Visualize
            self._visualize_current()
            
            # Show info about loaded data
            info_parts = [f"Loaded {len(self.original_xyz):,} points"]
            if self.original_colors is not None:
                info_parts.append("with RGB colors")
            if self.original_intensity is not None:
                info_parts.append("with intensity")
            if self.original_normals is not None:
                info_parts.append("with normals")
            
            QMessageBox.information(self, "Success", ", ".join(info_parts))
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load LAS file:\n{str(e)}")
            
    def reset_to_original(self):
        """Reset to original data"""
        if self.original_xyz is None:
            return
            
        self.current_xyz = self.original_xyz.copy()
        self.current_colors = self.original_colors.copy() if self.original_colors is not None else None
        self.current_intensity = self.original_intensity.copy() if self.original_intensity is not None else None
        self.current_normals = self.original_normals.copy() if self.original_normals is not None else None
        
        self.undo_stack.clear()
        self._visualize_current()
        
    def save_las_file(self):
        """Save the current filtered point cloud to a LAS file"""
        if self.current_xyz is None:
            QMessageBox.warning(self, "Warning", "No point cloud to save")
            return
            
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save LAS File",
            "",
            "LAS Files (*.las);;LAZ Files (*.laz);;All Files (*)"
        )
        
        if not filename:
            return
            
        try:
            # Determine point format based on available data
            # Format 2: XYZ + RGB
            # Format 3: XYZ + RGB + Time (we'll use for RGB + Intensity)
            # Format 7: XYZ + RGB + Intensity (LAS 1.4)
            has_colors = self.current_colors is not None
            has_intensity = self.current_intensity is not None
            
            if has_colors and has_intensity:
                point_format = 3  # Supports XYZ, Intensity, and RGB
                version = "1.2"
            elif has_colors:
                point_format = 2  # Supports XYZ and RGB
                version = "1.2"
            elif has_intensity:
                point_format = 1  # Supports XYZ and Intensity
                version = "1.2"
            else:
                point_format = 0  # Basic XYZ
                version = "1.2"
            
            # Create header
            header = laspy.LasHeader(point_format=point_format, version=version)
            header.offsets = np.min(self.current_xyz, axis=0)
            header.scales = [0.001, 0.001, 0.001]
            
            # Create LAS object
            las = laspy.LasData(header)
            
            # Set coordinates
            las.x = self.current_xyz[:, 0]
            las.y = self.current_xyz[:, 1]
            las.z = self.current_xyz[:, 2]
            
            # Set intensity if available
            if self.current_intensity is not None:
                las.intensity = self.current_intensity.astype(np.uint16)
            
            # Set colors if available
            if self.current_colors is not None:
                # Ensure colors are in 0-65535 range (16-bit)
                if self.current_colors.max() <= 255:
                    # Scale from 8-bit to 16-bit
                    colors_16bit = (self.current_colors.astype(np.uint32) * 257).astype(np.uint16)
                else:
                    colors_16bit = self.current_colors.astype(np.uint16)
                    
                las.red = colors_16bit[:, 0]
                las.green = colors_16bit[:, 1]
                las.blue = colors_16bit[:, 2]
            
            # Set normals if available (as extra bytes)
            if self.current_normals is not None:
                try:
                    # Add extra bytes for normals
                    las.add_extra_dim(laspy.ExtraBytesParams(
                        name="NormalX",
                        type=np.float32
                    ))
                    las.add_extra_dim(laspy.ExtraBytesParams(
                        name="NormalY",
                        type=np.float32
                    ))
                    las.add_extra_dim(laspy.ExtraBytesParams(
                        name="NormalZ",
                        type=np.float32
                    ))
                    
                    las.NormalX = self.current_normals[:, 0].astype(np.float32)
                    las.NormalY = self.current_normals[:, 1].astype(np.float32)
                    las.NormalZ = self.current_normals[:, 2].astype(np.float32)
                except Exception as e:
                    print(f"Warning: Could not save normals: {e}")
                
            las.write(filename)
            
            # Show what was saved
            info_parts = [f"Saved {len(self.current_xyz):,} points"]
            if self.current_colors is not None:
                info_parts.append("with RGB colors")
            if self.current_intensity is not None:
                info_parts.append("with intensity")
            if self.current_normals is not None:
                info_parts.append("with normals")
            
            QMessageBox.information(
                self, 
                "Success", 
                f"{', '.join(info_parts)} to:\n{filename}"
            )
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save LAS file:\n{str(e)}")
        
    # ========================
    # Filtering operations
    # ========================
    
    def apply_downsampling(self):
        """Apply voxel downsampling"""
        if self.current_xyz is None:
            return
            
        voxel_size = self.spin_voxel.value()
        
        # Store old state
        old_data = (
            self.current_xyz.copy(),
            self.current_colors.copy() if self.current_colors is not None else None,
            self.current_intensity.copy() if self.current_intensity is not None else None,
            self.current_normals.copy() if self.current_normals is not None else None
        )
        
        # Apply downsampling
        xyz_down, colors_down, intensity_down, normals_down = self._voxel_downsample(
            self.current_xyz,
            self.current_colors,
            self.current_intensity,
            self.current_normals,
            voxel_size
        )
        
        # Store new state
        new_data = (xyz_down, colors_down, intensity_down, normals_down)
        
        # Create undo command
        command = FilterCommand(self, old_data, new_data, f"Downsample (voxel={voxel_size}m)")
        self.undo_stack.push(command)
        
    def apply_denoising(self):
        """Apply statistical outlier removal"""
        if self.current_xyz is None:
            return
            
        n_neighbors = self.spin_neighbors.value()
        std_multiplier = self.spin_std.value()
        
        # Store old state
        old_data = (
            self.current_xyz.copy(),
            self.current_colors.copy() if self.current_colors is not None else None,
            self.current_intensity.copy() if self.current_intensity is not None else None,
            self.current_normals.copy() if self.current_normals is not None else None
        )
        
        # Apply denoising
        xyz_clean, colors_clean, intensity_clean, normals_clean = self._statistical_outlier_removal(
            self.current_xyz,
            self.current_colors,
            self.current_intensity,
            self.current_normals,
            n_neighbors,
            std_multiplier
        )
        
        # Store new state
        new_data = (xyz_clean, colors_clean, intensity_clean, normals_clean)
        
        # Create undo command
        command = FilterCommand(
            self, old_data, new_data,
            f"Denoise (neighbors={n_neighbors}, std={std_multiplier})"
        )
        self.undo_stack.push(command)
        
    def apply_height_filter(self):
        """Apply height-based filtering"""
        if self.current_xyz is None:
            return
            
        min_h = self.spin_min_height.value()
        max_h = self.spin_max_height.value()
        
        # Store old state
        old_data = (
            self.current_xyz.copy(),
            self.current_colors.copy() if self.current_colors is not None else None,
            self.current_intensity.copy() if self.current_intensity is not None else None,
            self.current_normals.copy() if self.current_normals is not None else None
        )
        
        # Apply filter
        mask = (self.current_xyz[:, 2] >= min_h) & (self.current_xyz[:, 2] <= max_h)
        xyz_filt = self.current_xyz[mask]
        colors_filt = self.current_colors[mask] if self.current_colors is not None else None
        intensity_filt = self.current_intensity[mask] if self.current_intensity is not None else None
        normals_filt = self.current_normals[mask] if self.current_normals is not None else None
        
        # Store new state
        new_data = (xyz_filt, colors_filt, intensity_filt, normals_filt)
        
        # Create undo command
        command = FilterCommand(self, old_data, new_data, f"Height Filter ({min_h}m to {max_h}m)")
        self.undo_stack.push(command)
        
    # ========================
    # Core filtering algorithms
    # ========================
    
    def _voxel_downsample(self, xyz, colors, intensity, normals, voxel_size):
        """Downsample point cloud using voxel grid"""
        # Compute voxel indices
        voxel_indices = np.floor(xyz / voxel_size).astype(np.int32)
        
        # Find unique voxels
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        
        xyz_down = xyz[unique_indices]
        colors_down = colors[unique_indices] if colors is not None else None
        intensity_down = intensity[unique_indices] if intensity is not None else None
        normals_down = normals[unique_indices] if normals is not None else None
        
        return xyz_down, colors_down, intensity_down, normals_down
        
    def _statistical_outlier_removal(self, xyz, colors, intensity, normals, n_neighbors, std_multiplier):
        """Remove statistical outliers using KD-tree"""
        if len(xyz) < n_neighbors:
            return xyz, colors, intensity, normals
            
        # Build KD-tree
        tree = cKDTree(xyz)
        
        # Find k-nearest neighbors for each point
        distances, _ = tree.query(xyz, k=n_neighbors + 1)
        
        # Compute mean distance (excluding self)
        mean_distances = distances[:, 1:].mean(axis=1)
        
        # Compute threshold
        global_mean = mean_distances.mean()
        global_std = mean_distances.std()
        threshold = global_mean + std_multiplier * global_std
        
        # Filter points
        mask = mean_distances < threshold
        
        xyz_clean = xyz[mask]
        colors_clean = colors[mask] if colors is not None else None
        intensity_clean = intensity[mask] if intensity is not None else None
        normals_clean = normals[mask] if normals is not None else None
        
        return xyz_clean, colors_clean, intensity_clean, normals_clean
        
    # ========================
    # Visualization
    # ========================
    
    def _apply_data(self, data):
        """Apply data state (for undo/redo)"""
        self.current_xyz, self.current_colors, self.current_intensity, self.current_normals = data
        self._visualize_current()
        
    def _visualize_current(self):
        """Visualize current point cloud"""
        # Clear previous actors
        if self.point_actor:
            self.plotter.remove_actor(self.point_actor)
            
        if self.current_xyz is None or len(self.current_xyz) == 0:
            self.lbl_point_count.setText("Points: 0")
            return
            
        # Create point cloud
        cloud = pv.PolyData(self.current_xyz)
        
        # Add colors if available
        if self.current_colors is not None and len(self.current_colors) > 0:
            cloud['colors'] = self.current_colors
            self.point_actor = self.plotter.add_mesh(
                cloud,
                scalars='colors',
                rgb=True,
                point_size=2,
                render_points_as_spheres=True
            )
        else:
            self.point_actor = self.plotter.add_mesh(
                cloud,
                color='lightblue',
                point_size=2,
                render_points_as_spheres=True
            )
            
        # Update point count
        self.lbl_point_count.setText(f"Points: {len(self.current_xyz):,}")
        
        # Reset camera on first load
        if not hasattr(self, '_camera_set'):
            self.plotter.reset_camera()
            self._camera_set = True


if __name__ == "__main__":
    from PySide6.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    viewer = PointCloudFilterViewer()
    viewer.show()
    sys.exit(app.exec())
