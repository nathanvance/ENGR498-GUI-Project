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
        # old_data and new_data are tuples of (xyz, colors)
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
    Features real-time preview and undo/redo functionality.
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Point Cloud Filter Viewer")
        self.resize(1400, 800)
        
        # Data storage
        self.original_xyz = None
        self.original_colors = None
        
        self.current_xyz = None
        self.current_colors = None
        
        self.preview_xyz = None
        self.preview_colors = None
        
        # Actor references
        self.point_actor = None
        self.preview_actor = None
        
        # Preview state
        self.preview_active = False
        self.preview_timer = QTimer()
        self.preview_timer.setSingleShot(True)
        self.preview_timer.timeout.connect(self._update_preview)
        
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
        
        # Preview toggle
        self.cb_preview = QCheckBox("Enable Real-time Preview")
        self.cb_preview.setChecked(True)
        self.cb_preview.stateChanged.connect(self._toggle_preview)
        layout.addWidget(self.cb_preview)
        
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
        self.slider_voxel.valueChanged.connect(self._on_slider_changed)
        slider_layout.addWidget(self.slider_voxel)
        
        self.spin_voxel = QDoubleSpinBox()
        self.spin_voxel.setMinimum(0.01)
        self.spin_voxel.setMaximum(2.0)
        self.spin_voxel.setSingleStep(0.01)
        self.spin_voxel.setValue(0.1)
        self.spin_voxel.valueChanged.connect(self._on_voxel_spin_changed)
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
        self.slider_neighbors.valueChanged.connect(self._on_slider_changed)
        slider_layout1.addWidget(self.slider_neighbors)
        
        self.spin_neighbors = QSpinBox()
        self.spin_neighbors.setMinimum(5)
        self.spin_neighbors.setMaximum(100)
        self.spin_neighbors.setValue(20)
        self.spin_neighbors.valueChanged.connect(self._sync_neighbors_slider)
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
        self.slider_std.valueChanged.connect(self._on_slider_changed)
        slider_layout2.addWidget(self.slider_std)
        
        self.spin_std = QDoubleSpinBox()
        self.spin_std.setMinimum(1.0)
        self.spin_std.setMaximum(5.0)
        self.spin_std.setSingleStep(0.1)
        self.spin_std.setValue(2.0)
        self.spin_std.valueChanged.connect(self._sync_std_slider)
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
        self.slider_min_height.valueChanged.connect(self._on_slider_changed)
        slider_layout1.addWidget(self.slider_min_height)
        
        self.spin_min_height = QDoubleSpinBox()
        self.spin_min_height.setMinimum(-1000)
        self.spin_min_height.setMaximum(1000)
        self.spin_min_height.setValue(-1000)
        self.spin_min_height.valueChanged.connect(self._sync_min_height_slider)
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
        self.slider_max_height.valueChanged.connect(self._on_slider_changed)
        slider_layout2.addWidget(self.slider_max_height)
        
        self.spin_max_height = QDoubleSpinBox()
        self.spin_max_height.setMinimum(-1000)
        self.spin_max_height.setMaximum(1000)
        self.spin_max_height.setValue(1000)
        self.spin_max_height.valueChanged.connect(self._sync_max_height_slider)
        slider_layout2.addWidget(self.spin_max_height)
        
        layout.addLayout(slider_layout2)
        
        # Apply button
        btn_apply = QPushButton("Apply Height Filter")
        btn_apply.clicked.connect(self.apply_height_filter)
        layout.addWidget(btn_apply)
        
        return group
        
    # ========================
    # Slider synchronization
    # ========================
    
    def _on_slider_changed(self):
        """Generic slider changed handler - triggers preview update"""
        if self.preview_active and self.current_xyz is not None:
            # Debounce: wait 300ms after last change
            self.preview_timer.start(300)
            
    def _on_voxel_spin_changed(self, value):
        """Sync voxel size slider with spinbox"""
        self.slider_voxel.blockSignals(True)
        self.slider_voxel.setValue(int(value * 100))
        self.slider_voxel.blockSignals(False)
        self._on_slider_changed()
        
    def _sync_neighbors_slider(self, value):
        """Sync neighbors slider with spinbox"""
        self.slider_neighbors.blockSignals(True)
        self.slider_neighbors.setValue(value)
        self.slider_neighbors.blockSignals(False)
        self._on_slider_changed()
        
    def _sync_std_slider(self, value):
        """Sync std dev slider with spinbox"""
        self.slider_std.blockSignals(True)
        self.slider_std.setValue(int(value * 10))
        self.slider_std.blockSignals(False)
        self._on_slider_changed()
        
    def _sync_min_height_slider(self, value):
        """Sync min height slider with spinbox"""
        self.slider_min_height.blockSignals(True)
        self.slider_min_height.setValue(int(value / 10))
        self.slider_min_height.blockSignals(False)
        self._on_slider_changed()
        
    def _sync_max_height_slider(self, value):
        """Sync max height slider with spinbox"""
        self.slider_max_height.blockSignals(True)
        self.slider_max_height.setValue(int(value / 10))
        self.slider_max_height.blockSignals(False)
        self._on_slider_changed()
        
    # ========================
    # Preview functionality
    # ========================
    
    def _toggle_preview(self, state):
        """Toggle preview mode"""
        self.preview_active = (state == Qt.Checked)
        if not self.preview_active and self.preview_actor:
            self.plotter.remove_actor(self.preview_actor)
            self.preview_actor = None
            
    def _update_preview(self):
        """Update the preview with current slider values"""
        if not self.preview_active or self.current_xyz is None:
            return
            
        # Work on a downsampled version for performance
        max_preview_points = 50000
        if len(self.current_xyz) > max_preview_points:
            indices = np.random.choice(len(self.current_xyz), max_preview_points, replace=False)
            preview_xyz = self.current_xyz[indices]
            preview_colors = self.current_colors[indices] if self.current_colors is not None else None
        else:
            preview_xyz = self.current_xyz
            preview_colors = self.current_colors
            
        # Apply current filter settings
        filtered_xyz, filtered_colors = self._apply_current_filters(preview_xyz, preview_colors)
        
        # Update preview visualization
        self._show_preview(filtered_xyz, filtered_colors)
        
    def _apply_current_filters(self, xyz, colors):
        """Apply all current filter settings to given data"""
        # Height filter
        min_h = self.spin_min_height.value()
        max_h = self.spin_max_height.value()
        mask = (xyz[:, 2] >= min_h) & (xyz[:, 2] <= max_h)
        xyz = xyz[mask]
        if colors is not None:
            colors = colors[mask]
            
        return xyz, colors
        
    def _show_preview(self, xyz, colors):
        """Display preview points in the viewer"""
        if self.preview_actor:
            self.plotter.remove_actor(self.preview_actor)
            
        if len(xyz) == 0:
            self.preview_actor = None
            return
            
        cloud = pv.PolyData(xyz)
        
        if colors is not None and len(colors) > 0:
            cloud['colors'] = colors
            self.preview_actor = self.plotter.add_mesh(
                cloud,
                scalars='colors',
                rgb=True,
                point_size=3,
                opacity=0.7,
                render_points_as_spheres=True
            )
        else:
            self.preview_actor = self.plotter.add_mesh(
                cloud,
                color='yellow',
                point_size=3,
                opacity=0.7,
                render_points_as_spheres=True
            )
            
    # ========================
    # File operations
    # ========================
    
    def load_las_file(self):
        """Load a LAS file"""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open LAS File",
            "",
            "LAS Files (*.las *.laz);;All Files (*)"
        )
        
        if not filename:
            return
            
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
                
            # Set as current data
            self.current_xyz = self.original_xyz.copy()
            self.current_colors = self.original_colors.copy() if self.original_colors is not None else None
            
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
            
            QMessageBox.information(self, "Success", f"Loaded {len(self.original_xyz):,} points")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load LAS file:\n{str(e)}")
            
    def reset_to_original(self):
        """Reset to original data"""
        if self.original_xyz is None:
            return
            
        self.current_xyz = self.original_xyz.copy()
        self.current_colors = self.original_colors.copy() if self.original_colors is not None else None
        
        self.undo_stack.clear()
        self._visualize_current()
        
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
            self.current_colors.copy() if self.current_colors is not None else None
        )
        
        # Apply downsampling
        xyz_down, colors_down = self._voxel_downsample(
            self.current_xyz,
            self.current_colors,
            voxel_size
        )
        
        # Store new state
        new_data = (xyz_down, colors_down)
        
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
            self.current_colors.copy() if self.current_colors is not None else None
        )
        
        # Apply denoising
        xyz_clean, colors_clean = self._statistical_outlier_removal(
            self.current_xyz,
            self.current_colors,
            n_neighbors,
            std_multiplier
        )
        
        # Store new state
        new_data = (xyz_clean, colors_clean)
        
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
            self.current_colors.copy() if self.current_colors is not None else None
        )
        
        # Apply filter
        mask = (self.current_xyz[:, 2] >= min_h) & (self.current_xyz[:, 2] <= max_h)
        xyz_filt = self.current_xyz[mask]
        colors_filt = self.current_colors[mask] if self.current_colors is not None else None
        
        # Store new state
        new_data = (xyz_filt, colors_filt)
        
        # Create undo command
        command = FilterCommand(self, old_data, new_data, f"Height Filter ({min_h}m to {max_h}m)")
        self.undo_stack.push(command)
        
    # ========================
    # Core filtering algorithms
    # ========================
    
    def _voxel_downsample(self, xyz, colors, voxel_size):
        """Downsample point cloud using voxel grid"""
        # Compute voxel indices
        voxel_indices = np.floor(xyz / voxel_size).astype(np.int32)
        
        # Find unique voxels
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        
        xyz_down = xyz[unique_indices]
        colors_down = colors[unique_indices] if colors is not None else None
        
        return xyz_down, colors_down
        
    def _statistical_outlier_removal(self, xyz, colors, n_neighbors, std_multiplier):
        """Remove statistical outliers using KD-tree"""
        if len(xyz) < n_neighbors:
            return xyz, colors
            
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
        
        return xyz_clean, colors_clean
        
    # ========================
    # Visualization
    # ========================
    
    def _apply_data(self, data):
        """Apply data state (for undo/redo)"""
        self.current_xyz, self.current_colors = data
        self._visualize_current()
        
    def _visualize_current(self):
        """Visualize current point cloud"""
        # Clear previous actors
        if self.point_actor:
            self.plotter.remove_actor(self.point_actor)
        if self.preview_actor:
            self.plotter.remove_actor(self.preview_actor)
            self.preview_actor = None
            
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