import numpy as np
import laspy
import pyvista as pv
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel,
    QSlider, QSpinBox, QDoubleSpinBox, QGroupBox, QCheckBox,
    QSplitter, QFileDialog, QMessageBox, QScrollArea
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
        self.old_data = old_data
        self.new_data = new_data
        
    def undo(self):
        self.viewer._apply_data(self.old_data)
        
    def redo(self):
        self.viewer._apply_data(self.new_data)


class PointCloudFilterViewer(QWidget):
    backRequested = Signal()  # Signal to notify when user wants to go back to dashboard
    """
    Point cloud viewer with filtering, downsampling, and denoising capabilities.
    Features undo/redo functionality and preserves all point attributes.
    """
    
    def __init__(self, parent=None, filename=None):
        super().__init__(parent)
        self.setWindowTitle("Point Cloud Filter Viewer")
        self.resize(1400, 800)

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
        self.crop_box_actor = None

        # Suppress crop spinbox feedback loops
        self._updating_crop_ui = False
        
        # Undo stack
        self.undo_stack = QUndoStack(self)
        
        self._setup_ui()
        
    def _setup_ui(self):
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)
        
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        viewer_widget = self._create_viewer_widget()
        splitter.addWidget(viewer_widget)
        
        control_widget = self._create_control_panel()
        splitter.addWidget(control_widget)
        
        splitter.setSizes([900, 500])
        
    def _create_viewer_widget(self):
        widget = QWidget()
        layout = QVBoxLayout()
        widget.setLayout(layout)
        
        self.plotter = QtInteractor(self)
        self.plotter.setMinimumSize(800, 600)
        layout.addWidget(self.plotter)
        
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
        
        btn_save = QPushButton("Save LAS and Return")
        btn_save.clicked.connect(self.save_las_file)
        controls_layout.addWidget(btn_save)

        btn_cancel = QPushButton("Cancel")
        btn_cancel.clicked.connect(self.backRequested)
        controls_layout.addWidget(btn_cancel)

        controls_layout.addStretch()
        
        self.lbl_point_count = QLabel("Points: 0")
        controls_layout.addWidget(self.lbl_point_count)
        
        layout.addLayout(controls_layout)
        
        return widget
        
    def _create_control_panel(self):
        outer_widget = QWidget()
        outer_layout = QVBoxLayout()
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_widget.setLayout(outer_layout)

        title = QLabel("Filtering Controls")
        title.setStyleSheet("font-weight: bold; font-size: 16px; padding: 8px;")
        outer_layout.addWidget(title)

        # Scroll area so the panel never gets cramped
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        outer_layout.addWidget(scroll)

        inner_widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        inner_widget.setLayout(layout)
        scroll.setWidget(inner_widget)

        layout.addWidget(self._create_downsample_group())
        layout.addWidget(self._create_sor_group())
        layout.addWidget(self._create_ror_group())
        layout.addWidget(self._create_sla_group())
        layout.addWidget(self._create_crop_group())
        layout.addStretch()
        
        return outer_widget
        
    # ------------------------------------------------------------------
    # Control group builders
    # ------------------------------------------------------------------

    def _create_downsample_group(self):
        group = QGroupBox("Downsampling")
        layout = QVBoxLayout()
        group.setLayout(layout)
        
        layout.addWidget(QLabel("Voxel Size (m):"))
        
        row = QHBoxLayout()
        self.slider_voxel = QSlider(Qt.Horizontal)
        self.slider_voxel.setMinimum(1)
        self.slider_voxel.setMaximum(200)
        self.slider_voxel.setValue(10)
        row.addWidget(self.slider_voxel)
        
        self.spin_voxel = QDoubleSpinBox()
        self.spin_voxel.setMinimum(0.01)
        self.spin_voxel.setMaximum(2.0)
        self.spin_voxel.setSingleStep(0.01)
        self.spin_voxel.setValue(0.1)
        self.slider_voxel.valueChanged.connect(lambda v: self.spin_voxel.setValue(v / 100.0))
        self.spin_voxel.valueChanged.connect(lambda v: self.slider_voxel.setValue(int(v * 100)))
        row.addWidget(self.spin_voxel)
        layout.addLayout(row)
        
        btn = QPushButton("Apply Downsampling")
        btn.clicked.connect(self.apply_downsampling)
        layout.addWidget(btn)
        
        return group

    def _create_sor_group(self):
        """Statistical Outlier Removal"""
        group = QGroupBox("Statistical Outlier Removal")
        layout = QVBoxLayout()
        group.setLayout(layout)
        
        layout.addWidget(QLabel("Number of Neighbors:"))
        row1 = QHBoxLayout()
        self.slider_sor_neighbors = QSlider(Qt.Horizontal)
        self.slider_sor_neighbors.setMinimum(5)
        self.slider_sor_neighbors.setMaximum(100)
        self.slider_sor_neighbors.setValue(20)
        row1.addWidget(self.slider_sor_neighbors)
        self.spin_sor_neighbors = QSpinBox()
        self.spin_sor_neighbors.setMinimum(5)
        self.spin_sor_neighbors.setMaximum(100)
        self.spin_sor_neighbors.setValue(20)
        self.slider_sor_neighbors.valueChanged.connect(self.spin_sor_neighbors.setValue)
        self.spin_sor_neighbors.valueChanged.connect(self.slider_sor_neighbors.setValue)
        row1.addWidget(self.spin_sor_neighbors)
        layout.addLayout(row1)
        
        layout.addWidget(QLabel("Std Dev Multiplier:"))
        row2 = QHBoxLayout()
        self.slider_sor_std = QSlider(Qt.Horizontal)
        self.slider_sor_std.setMinimum(10)
        self.slider_sor_std.setMaximum(50)
        self.slider_sor_std.setValue(20)
        row2.addWidget(self.slider_sor_std)
        self.spin_sor_std = QDoubleSpinBox()
        self.spin_sor_std.setMinimum(1.0)
        self.spin_sor_std.setMaximum(5.0)
        self.spin_sor_std.setSingleStep(0.1)
        self.spin_sor_std.setValue(2.0)
        self.slider_sor_std.valueChanged.connect(lambda v: self.spin_sor_std.setValue(v / 10.0))
        self.spin_sor_std.valueChanged.connect(lambda v: self.slider_sor_std.setValue(int(v * 10)))
        row2.addWidget(self.spin_sor_std)
        layout.addLayout(row2)
        
        btn = QPushButton("Apply Statistical Outlier Removal")
        btn.clicked.connect(self.apply_sor)
        layout.addWidget(btn)
        
        return group

    def _create_ror_group(self):
        """Radius Outlier Removal"""
        group = QGroupBox("Radius Outlier Removal")
        layout = QVBoxLayout()
        group.setLayout(layout)

        lbl_desc = QLabel(
            "Removes points that have fewer than a minimum number of\n"
            "neighbors within a given search radius."
        )
        lbl_desc.setWordWrap(True)
        lbl_desc.setStyleSheet("color: gray; font-size: 11px;")
        layout.addWidget(lbl_desc)

        layout.addWidget(QLabel("Search Radius (m):"))
        row1 = QHBoxLayout()
        self.slider_ror_radius = QSlider(Qt.Horizontal)
        self.slider_ror_radius.setMinimum(1)
        self.slider_ror_radius.setMaximum(500)   # 0.01 – 5.00 m
        self.slider_ror_radius.setValue(50)       # 0.50 m default
        row1.addWidget(self.slider_ror_radius)
        self.spin_ror_radius = QDoubleSpinBox()
        self.spin_ror_radius.setMinimum(0.01)
        self.spin_ror_radius.setMaximum(5.0)
        self.spin_ror_radius.setSingleStep(0.01)
        self.spin_ror_radius.setValue(0.50)
        self.slider_ror_radius.valueChanged.connect(lambda v: self.spin_ror_radius.setValue(v / 100.0))
        self.spin_ror_radius.valueChanged.connect(lambda v: self.slider_ror_radius.setValue(int(v * 100)))
        row1.addWidget(self.spin_ror_radius)
        layout.addLayout(row1)

        layout.addWidget(QLabel("Min Neighbors in Radius:"))
        row2 = QHBoxLayout()
        self.slider_ror_min_neighbors = QSlider(Qt.Horizontal)
        self.slider_ror_min_neighbors.setMinimum(1)
        self.slider_ror_min_neighbors.setMaximum(50)
        self.slider_ror_min_neighbors.setValue(5)
        row2.addWidget(self.slider_ror_min_neighbors)
        self.spin_ror_min_neighbors = QSpinBox()
        self.spin_ror_min_neighbors.setMinimum(1)
        self.spin_ror_min_neighbors.setMaximum(50)
        self.spin_ror_min_neighbors.setValue(5)
        self.slider_ror_min_neighbors.valueChanged.connect(self.spin_ror_min_neighbors.setValue)
        self.spin_ror_min_neighbors.valueChanged.connect(self.slider_ror_min_neighbors.setValue)
        row2.addWidget(self.spin_ror_min_neighbors)
        layout.addLayout(row2)

        btn = QPushButton("Apply Radius Outlier Removal")
        btn.clicked.connect(self.apply_ror)
        layout.addWidget(btn)

        return group

    def _create_sla_group(self):
        """Simple Local Averaging (positional smoothing)"""
        group = QGroupBox("Simple Local Averaging (Smoothing)")
        layout = QVBoxLayout()
        group.setLayout(layout)

        lbl_desc = QLabel(
            "Moves each point toward the average position of its k nearest\n"
            "neighbors. Strength 0 = no change, 1 = fully move to average."
        )
        lbl_desc.setWordWrap(True)
        lbl_desc.setStyleSheet("color: gray; font-size: 11px;")
        layout.addWidget(lbl_desc)

        layout.addWidget(QLabel("Number of Neighbors (k):"))
        row1 = QHBoxLayout()
        self.slider_sla_k = QSlider(Qt.Horizontal)
        self.slider_sla_k.setMinimum(3)
        self.slider_sla_k.setMaximum(50)
        self.slider_sla_k.setValue(10)
        row1.addWidget(self.slider_sla_k)
        self.spin_sla_k = QSpinBox()
        self.spin_sla_k.setMinimum(3)
        self.spin_sla_k.setMaximum(50)
        self.spin_sla_k.setValue(10)
        self.slider_sla_k.valueChanged.connect(self.spin_sla_k.setValue)
        self.spin_sla_k.valueChanged.connect(self.slider_sla_k.setValue)
        row1.addWidget(self.spin_sla_k)
        layout.addLayout(row1)

        layout.addWidget(QLabel("Smoothing Strength (0–1):"))
        row2 = QHBoxLayout()
        self.slider_sla_strength = QSlider(Qt.Horizontal)
        self.slider_sla_strength.setMinimum(0)
        self.slider_sla_strength.setMaximum(100)
        self.slider_sla_strength.setValue(50)
        row2.addWidget(self.slider_sla_strength)
        self.spin_sla_strength = QDoubleSpinBox()
        self.spin_sla_strength.setMinimum(0.0)
        self.spin_sla_strength.setMaximum(1.0)
        self.spin_sla_strength.setSingleStep(0.05)
        self.spin_sla_strength.setValue(0.5)
        self.slider_sla_strength.valueChanged.connect(lambda v: self.spin_sla_strength.setValue(v / 100.0))
        self.spin_sla_strength.valueChanged.connect(lambda v: self.slider_sla_strength.setValue(int(v * 100)))
        row2.addWidget(self.spin_sla_strength)
        layout.addLayout(row2)

        btn = QPushButton("Apply Local Averaging")
        btn.clicked.connect(self.apply_sla)
        layout.addWidget(btn)

        return group

    def _create_crop_group(self):
        """Bounding box crop with live preview in viewer"""
        group = QGroupBox("Crop Filter (Bounding Box)")
        layout = QVBoxLayout()
        group.setLayout(layout)

        lbl_desc = QLabel(
            "Set the bounding box below. A preview box is drawn in the\n"
            "viewer. Click 'Apply Crop' to keep only points inside."
        )
        lbl_desc.setWordWrap(True)
        lbl_desc.setStyleSheet("color: gray; font-size: 11px;")
        layout.addWidget(lbl_desc)

        # Helper to build one axis row (min + max spin boxes)
        def axis_row(label_text, attr_min, attr_max, lo, hi):
            layout.addWidget(QLabel(label_text))
            row = QHBoxLayout()

            spin_min = QDoubleSpinBox()
            spin_min.setMinimum(-1e7)
            spin_min.setMaximum(1e7)
            spin_min.setDecimals(3)
            spin_min.setSingleStep(0.1)
            spin_min.setValue(lo)
            spin_min.setPrefix("min: ")
            row.addWidget(spin_min)

            spin_max = QDoubleSpinBox()
            spin_max.setMinimum(-1e7)
            spin_max.setMaximum(1e7)
            spin_max.setDecimals(3)
            spin_max.setSingleStep(0.1)
            spin_max.setValue(hi)
            spin_max.setPrefix("max: ")
            row.addWidget(spin_max)

            layout.addLayout(row)
            setattr(self, attr_min, spin_min)
            setattr(self, attr_max, spin_max)

            # Live preview update
            spin_min.valueChanged.connect(self._update_crop_preview)
            spin_max.valueChanged.connect(self._update_crop_preview)

        axis_row("X bounds (m):", "spin_crop_xmin", "spin_crop_xmax", 0.0, 1.0)
        axis_row("Y bounds (m):", "spin_crop_ymin", "spin_crop_ymax", 0.0, 1.0)
        axis_row("Z bounds (m):", "spin_crop_zmin", "spin_crop_zmax", 0.0, 1.0)

        row_btns = QHBoxLayout()
        btn_reset_crop = QPushButton("Reset to Data Bounds")
        btn_reset_crop.clicked.connect(self._reset_crop_to_data_bounds)
        row_btns.addWidget(btn_reset_crop)

        btn_apply_crop = QPushButton("Apply Crop")
        btn_apply_crop.clicked.connect(self.apply_crop)
        row_btns.addWidget(btn_apply_crop)
        layout.addLayout(row_btns)

        return group

    # ------------------------------------------------------------------
    # File operations
    # ------------------------------------------------------------------
    
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
            
            self.original_xyz = np.vstack((las.x, las.y, las.z)).T
            
            if hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue'):
                colors = np.vstack((las.red, las.green, las.blue)).T
                if colors.max() > 255:
                    colors = (colors / 65535 * 255).astype(np.uint8)
                self.original_colors = colors
            else:
                self.original_colors = None
            
            if hasattr(las, 'intensity'):
                self.original_intensity = np.array(las.intensity)
            else:
                self.original_intensity = None
                
            if hasattr(las, 'NormalX') and hasattr(las, 'NormalY') and hasattr(las, 'NormalZ'):
                self.original_normals = np.vstack((las.NormalX, las.NormalY, las.NormalZ)).T
            else:
                self.original_normals = None
                
            self.current_xyz = self.original_xyz.copy()
            self.current_colors = self.original_colors.copy() if self.original_colors is not None else None
            self.current_intensity = self.original_intensity.copy() if self.original_intensity is not None else None
            self.current_normals = self.original_normals.copy() if self.original_normals is not None else None
            
            self.undo_stack.clear()

            self._reset_crop_to_data_bounds()
            self._visualize_current()
            
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
        if self.original_xyz is None:
            return
            
        self.current_xyz = self.original_xyz.copy()
        self.current_colors = self.original_colors.copy() if self.original_colors is not None else None
        self.current_intensity = self.original_intensity.copy() if self.original_intensity is not None else None
        self.current_normals = self.original_normals.copy() if self.original_normals is not None else None
        
        self.undo_stack.clear()
        self._reset_crop_to_data_bounds()
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
        
    # ------------------------------------------------------------------
    # Filter apply methods  (each pushes an undo command)
    # ------------------------------------------------------------------

    def _capture_state(self):
        """Return a snapshot of the current data (deep copy)."""
        return (
            self.current_xyz.copy(),
            self.current_colors.copy() if self.current_colors is not None else None,
            self.current_intensity.copy() if self.current_intensity is not None else None,
            self.current_normals.copy() if self.current_normals is not None else None,
        )

    def _push_filter(self, new_xyz, new_colors, new_intensity, new_normals, name):
        """Create & push an undo command that swaps to the supplied data."""
        old_data = self._capture_state()
        new_data = (new_xyz, new_colors, new_intensity, new_normals)
        cmd = FilterCommand(self, old_data, new_data, name)
        self.undo_stack.push(cmd)

    def apply_downsampling(self):
        if self.current_xyz is None:
            return
        voxel_size = self.spin_voxel.value()
        result = self._voxel_downsample(
            self.current_xyz, self.current_colors,
            self.current_intensity, self.current_normals, voxel_size
        )
        self._push_filter(*result, f"Downsample (voxel={voxel_size}m)")

    def apply_sor(self):
        """Statistical Outlier Removal"""
        if self.current_xyz is None:
            return
        n = self.spin_sor_neighbors.value()
        std = self.spin_sor_std.value()
        result = self._statistical_outlier_removal(
            self.current_xyz, self.current_colors,
            self.current_intensity, self.current_normals, n, std
        )
        self._push_filter(*result, f"SOR (k={n}, std={std})")

    def apply_ror(self):
        """Radius Outlier Removal"""
        if self.current_xyz is None:
            return
        radius = self.spin_ror_radius.value()
        min_nb = self.spin_ror_min_neighbors.value()
        result = self._radius_outlier_removal(
            self.current_xyz, self.current_colors,
            self.current_intensity, self.current_normals, radius, min_nb
        )
        self._push_filter(*result, f"ROR (r={radius}m, min_nb={min_nb})")

    def apply_sla(self):
        """Simple Local Averaging"""
        if self.current_xyz is None:
            return
        k = self.spin_sla_k.value()
        strength = self.spin_sla_strength.value()
        result = self._simple_local_averaging(
            self.current_xyz, self.current_colors,
            self.current_intensity, self.current_normals, k, strength
        )
        self._push_filter(*result, f"SLA (k={k}, strength={strength:.2f})")

    def apply_crop(self):
        """Crop to bounding box"""
        if self.current_xyz is None:
            return
        xmin = self.spin_crop_xmin.value()
        xmax = self.spin_crop_xmax.value()
        ymin = self.spin_crop_ymin.value()
        ymax = self.spin_crop_ymax.value()
        zmin = self.spin_crop_zmin.value()
        zmax = self.spin_crop_zmax.value()

        mask = (
            (self.current_xyz[:, 0] >= xmin) & (self.current_xyz[:, 0] <= xmax) &
            (self.current_xyz[:, 1] >= ymin) & (self.current_xyz[:, 1] <= ymax) &
            (self.current_xyz[:, 2] >= zmin) & (self.current_xyz[:, 2] <= zmax)
        )

        new_xyz   = self.current_xyz[mask]
        new_col   = self.current_colors[mask]    if self.current_colors    is not None else None
        new_int   = self.current_intensity[mask] if self.current_intensity is not None else None
        new_norm  = self.current_normals[mask]   if self.current_normals   is not None else None

        self._push_filter(
            new_xyz, new_col, new_int, new_norm,
            f"Crop X[{xmin:.2f},{xmax:.2f}] Y[{ymin:.2f},{ymax:.2f}] Z[{zmin:.2f},{zmax:.2f}]"
        )
        # Hide preview box after apply
        self._remove_crop_box()

    # ------------------------------------------------------------------
    # Core filtering algorithms
    # ------------------------------------------------------------------
    
    def _voxel_downsample(self, xyz, colors, intensity, normals, voxel_size):
        voxel_indices = np.floor(xyz / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        xyz_d     = xyz[unique_indices]
        colors_d  = colors[unique_indices]    if colors    is not None else None
        int_d     = intensity[unique_indices] if intensity is not None else None
        normals_d = normals[unique_indices]   if normals   is not None else None
        return xyz_d, colors_d, int_d, normals_d
        
    def _statistical_outlier_removal(self, xyz, colors, intensity, normals,
                                     n_neighbors, std_multiplier):
        if len(xyz) < n_neighbors:
            return xyz, colors, intensity, normals
        tree = cKDTree(xyz)
        distances, _ = tree.query(xyz, k=n_neighbors + 1)
        mean_distances = distances[:, 1:].mean(axis=1)
        threshold = mean_distances.mean() + std_multiplier * mean_distances.std()
        mask = mean_distances < threshold
        return (
            xyz[mask],
            colors[mask]    if colors    is not None else None,
            intensity[mask] if intensity is not None else None,
            normals[mask]   if normals   is not None else None,
        )

    def _radius_outlier_removal(self, xyz, colors, intensity, normals,
                                radius, min_neighbors):
        """
        Remove points that have fewer than `min_neighbors` within `radius`.
        Uses a ball query via cKDTree.
        """
        if len(xyz) == 0:
            return xyz, colors, intensity, normals

        tree = cKDTree(xyz)
        # query_ball_point returns neighbors including the point itself
        neighbor_counts = np.array([
            len(tree.query_ball_point(p, radius)) - 1   # exclude self
            for p in xyz
        ])
        mask = neighbor_counts >= min_neighbors
        return (
            xyz[mask],
            colors[mask]    if colors    is not None else None,
            intensity[mask] if intensity is not None else None,
            normals[mask]   if normals   is not None else None,
        )

    def _simple_local_averaging(self, xyz, colors, intensity, normals, k, strength):
        """
        Move each point toward the centroid of its k nearest neighbors.

        new_position = original + strength * (neighbor_centroid - original)

        Only XYZ coordinates are modified; attributes (colors, intensity,
        normals) are preserved unchanged.
        """
        if len(xyz) < k + 1:
            return xyz, colors, intensity, normals

        tree = cKDTree(xyz)
        # k+1 because the query includes the point itself at index 0
        _, indices = tree.query(xyz, k=k + 1)

        # Centroid of the k neighbors (excluding the point itself)
        neighbor_centroids = xyz[indices[:, 1:]].mean(axis=1)

        new_xyz = xyz + strength * (neighbor_centroids - xyz)

        return (
            new_xyz,
            colors.copy()    if colors    is not None else None,
            intensity.copy() if intensity is not None else None,
            normals.copy()   if normals   is not None else None,
        )

    # ------------------------------------------------------------------
    # Crop preview helpers
    # ------------------------------------------------------------------

    def _reset_crop_to_data_bounds(self):
        """Set crop spinboxes to the extents of the current point cloud."""
        if self.current_xyz is None:
            return
        self._updating_crop_ui = True
        try:
            mins = self.current_xyz.min(axis=0)
            maxs = self.current_xyz.max(axis=0)
            self.spin_crop_xmin.setValue(mins[0])
            self.spin_crop_xmax.setValue(maxs[0])
            self.spin_crop_ymin.setValue(mins[1])
            self.spin_crop_ymax.setValue(maxs[1])
            self.spin_crop_zmin.setValue(mins[2])
            self.spin_crop_zmax.setValue(maxs[2])
        finally:
            self._updating_crop_ui = False
        self._remove_crop_box()

    def _update_crop_preview(self):
        """Draw a wireframe bounding box in the viewer based on current spinbox values."""
        if self._updating_crop_ui or self.current_xyz is None:
            return

        xmin = self.spin_crop_xmin.value()
        xmax = self.spin_crop_xmax.value()
        ymin = self.spin_crop_ymin.value()
        ymax = self.spin_crop_ymax.value()
        zmin = self.spin_crop_zmin.value()
        zmax = self.spin_crop_zmax.value()

        # Remove old preview
        self._remove_crop_box()

        # Build box mesh and show as wireframe
        box = pv.Box(bounds=(xmin, xmax, ymin, ymax, zmin, zmax))
        self.crop_box_actor = self.plotter.add_mesh(
            box,
            style="wireframe",
            color="yellow",
            line_width=2,
            opacity=0.8,
            name="crop_preview"
        )

    def _remove_crop_box(self):
        if self.crop_box_actor is not None:
            try:
                self.plotter.remove_actor(self.crop_box_actor)
            except Exception:
                pass
            self.crop_box_actor = None

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    
    def _apply_data(self, data):
        """Apply data state (used by undo/redo)."""
        self.current_xyz, self.current_colors, self.current_intensity, self.current_normals = data
        self._visualize_current()
        
    def _visualize_current(self):
        if self.point_actor:
            self.plotter.remove_actor(self.point_actor)
            self.point_actor = None
            
        if self.current_xyz is None or len(self.current_xyz) == 0:
            self.lbl_point_count.setText("Points: 0")
            return
            
        cloud = pv.PolyData(self.current_xyz)
        
        if self.current_colors is not None and len(self.current_colors) > 0:
            cloud['colors'] = self.current_colors
            self.point_actor = self.plotter.add_mesh(
                cloud, scalars='colors', rgb=True,
                point_size=2, render_points_as_spheres=True
            )
        else:
            self.point_actor = self.plotter.add_mesh(
                cloud, color='lightblue',
                point_size=2, render_points_as_spheres=True
            )
            
        self.lbl_point_count.setText(f"Points: {len(self.current_xyz):,}")
        
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
