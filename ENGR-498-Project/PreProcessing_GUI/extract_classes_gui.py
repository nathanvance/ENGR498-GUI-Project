"""
GUI for LAS Classification Extractor

Interactive tool to extract specific classes from classified LAS point clouds.
"""

import numpy as np
import laspy
from pathlib import Path
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QCheckBox, QFileDialog, QMessageBox, QGroupBox, QScrollArea,
    QRadioButton, QButtonGroup, QLineEdit, QApplication
)
from PySide6.QtCore import Qt


# Classification mapping
CLASSIFICATION_MAP = {
    0: "Not classified",
    1: "Other",
    2: "Ground",
    3: "Vegetation",
    6: "Buildings",
    7: "Low noise",
    18: "High noise",
    21: "Vehicles",
    24: "Low voltage wire",
    25: "High voltage wire",
    26: "Railroad wire",
    27: "Low voltage tower",
    28: "High voltage tower",
    29: "Railroad tower",
    30: "Fences",
    31: "Insulator",
    32: "Guy wire",
    34: "Tension wire"
    #18: "Cross-arms",
    #19: "Pedestals"
}


class ClassificationExtractorGUI(QWidget):
    """GUI for extracting classifications from LAS files"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("LAS Classification Extractor")
        self.resize(700, 600)
        
        self.input_file = None
        self.las_data = None
        self.class_stats = {}
        
        self._setup_ui()
        
    def _setup_ui(self):
        """Setup the UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Title
        title = QLabel("LAS Classification Extractor")
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
        
        self.lbl_stats = QLabel("")
        file_layout.addWidget(self.lbl_stats)
        
        layout.addWidget(file_group)
        
        # Output format
        format_group = QGroupBox("Output Format")
        format_layout = QHBoxLayout()
        format_group.setLayout(format_layout)
        
        self.radio_las = QRadioButton("LAS file")
        self.radio_las.setChecked(True)
        format_layout.addWidget(self.radio_las)
        
        self.radio_npz = QRadioButton("NPZ file (NumPy)")
        format_layout.addWidget(self.radio_npz)
        
        format_layout.addStretch()
        layout.addWidget(format_group)
        
        # Quick selection buttons
        quick_group = QGroupBox("Quick Select")
        quick_layout = QVBoxLayout()
        quick_group.setLayout(quick_layout)
        
        quick_btn_layout1 = QHBoxLayout()
        
        btn_all_wires = QPushButton("All Wires")
        btn_all_wires.clicked.connect(lambda: self._quick_select([24, 25, 26, 32, 34]))
        quick_btn_layout1.addWidget(btn_all_wires)
        
        btn_towers = QPushButton("All Towers")
        btn_towers.clicked.connect(lambda: self._quick_select([27, 28, 29]))
        quick_btn_layout1.addWidget(btn_towers)
        
        btn_ground = QPushButton("Ground")
        btn_ground.clicked.connect(lambda: self._quick_select([2]))
        quick_btn_layout1.addWidget(btn_ground)
        
        quick_layout.addLayout(quick_btn_layout1)
        
        quick_btn_layout2 = QHBoxLayout()
        
        btn_select_all = QPushButton("Select All")
        btn_select_all.clicked.connect(self._select_all)
        quick_btn_layout2.addWidget(btn_select_all)
        
        btn_select_none = QPushButton("Select None")
        btn_select_none.clicked.connect(self._select_none)
        quick_btn_layout2.addWidget(btn_select_none)
        
        btn_invert = QPushButton("Invert Selection")
        btn_invert.clicked.connect(self._invert_selection)
        quick_btn_layout2.addWidget(btn_invert)
        
        quick_layout.addLayout(quick_btn_layout2)
        
        layout.addWidget(quick_group)
        
        # Class selection checkboxes
        class_group = QGroupBox("Select Classes to Extract")
        class_layout = QVBoxLayout()
        class_group.setLayout(class_layout)
        
        # Scrollable area for checkboxes
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        self.class_checkboxes_layout = QVBoxLayout()
        scroll_content.setLayout(self.class_checkboxes_layout)
        scroll.setWidget(scroll_content)
        
        class_layout.addWidget(scroll)
        layout.addWidget(class_group)
        
        self.class_checkboxes = {}
        self._create_class_checkboxes()
        
        # Extract button
        btn_layout = QHBoxLayout()
        
        self.btn_extract = QPushButton("Extract Selected Classes")
        self.btn_extract.setEnabled(False)
        self.btn_extract.clicked.connect(self.extract_classes)
        self.btn_extract.setStyleSheet("font-weight: bold; padding: 10px;")
        btn_layout.addWidget(self.btn_extract)
        
        layout.addLayout(btn_layout)
        
    def _create_class_checkboxes(self):
        """Create checkboxes for all classification types"""
        for class_id in sorted(CLASSIFICATION_MAP.keys()):
            class_name = CLASSIFICATION_MAP[class_id]
            
            cb_layout = QHBoxLayout()
            cb = QCheckBox(f"[{class_id:2d}] {class_name}")
            cb.setEnabled(False)  # Disabled until file is loaded
            
            # Add count label
            count_label = QLabel("")
            count_label.setMinimumWidth(150)
            count_label.setAlignment(Qt.AlignRight)
            
            cb_layout.addWidget(cb)
            cb_layout.addWidget(count_label)
            cb_layout.addStretch()
            
            self.class_checkboxes_layout.addLayout(cb_layout)
            self.class_checkboxes[class_id] = (cb, count_label)
            
    def browse_file(self):
        """Browse for input LAS file"""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open Classified LAS File",
            "",
            "LAS Files (*.las *.laz);;All Files (*)"
        )
        
        if filename:
            self.load_file(filename)
            
    def load_file(self, filename):
        """Load and analyze LAS file"""
        try:
            self.input_file = filename
            self.lbl_file.setText(Path(filename).name)
            
            # Load LAS
            self.las_data = laspy.read(filename)
            
            # Get classification statistics
            classifications = np.array(self.las_data.classification)
            unique, counts = np.unique(classifications, return_counts=True)
            
            self.class_stats = {}
            for class_id, count in zip(unique, counts):
                self.class_stats[class_id] = count
            
            # Update UI
            total_points = len(classifications)
            self.lbl_stats.setText(f"Total points: {total_points:,}")
            
            # Update checkboxes
            for class_id, (cb, count_label) in self.class_checkboxes.items():
                if class_id in self.class_stats:
                    count = self.class_stats[class_id]
                    percentage = (count / total_points) * 100
                    count_label.setText(f"{count:,} ({percentage:.1f}%)")
                    cb.setEnabled(True)
                    cb.setChecked(False)
                else:
                    count_label.setText("0 (0.0%)")
                    cb.setEnabled(False)
                    cb.setChecked(False)
            
            self.btn_extract.setEnabled(True)
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load LAS file:\n{str(e)}")
            
    def _quick_select(self, class_ids):
        """Quick select specific classes"""
        # First deselect all
        self._select_none()
        # Then select specified
        for class_id in class_ids:
            if class_id in self.class_checkboxes:
                cb, _ = self.class_checkboxes[class_id]
                if cb.isEnabled():
                    cb.setChecked(True)
                    
    def _select_all(self):
        """Select all available classes"""
        for cb, _ in self.class_checkboxes.values():
            if cb.isEnabled():
                cb.setChecked(True)
                
    def _select_none(self):
        """Deselect all classes"""
        for cb, _ in self.class_checkboxes.values():
            cb.setChecked(False)
            
    def _invert_selection(self):
        """Invert selection"""
        for cb, _ in self.class_checkboxes.values():
            if cb.isEnabled():
                cb.setChecked(not cb.isChecked())
                
    def extract_classes(self):
        """Extract selected classes"""
        if self.las_data is None:
            return
            
        # Get selected classes
        selected_classes = [
            class_id for class_id, (cb, _) in self.class_checkboxes.items()
            if cb.isChecked()
        ]
        
        if not selected_classes:
            QMessageBox.warning(self, "Warning", "No classes selected!")
            return
            
        # Choose output file
        if self.radio_npz.isChecked():
            output_file, _ = QFileDialog.getSaveFileName(
                self,
                "Save NPZ File",
                "",
                "NPZ Files (*.npz);;All Files (*)"
            )
        else:
            output_file, _ = QFileDialog.getSaveFileName(
                self,
                "Save LAS File",
                "",
                "LAS Files (*.las);;All Files (*)"
            )
            
        if not output_file:
            return
            
        try:
            # Extract points
            classifications = np.array(self.las_data.classification)
            mask = np.isin(classifications, selected_classes)
            
            xyz = np.vstack((
                self.las_data.x[mask],
                self.las_data.y[mask],
                self.las_data.z[mask]
            )).T
            
            # Extract colors if available
            colors = None
            if hasattr(self.las_data, 'red'):
                colors = np.vstack((
                    self.las_data.red[mask],
                    self.las_data.green[mask],
                    self.las_data.blue[mask]
                )).T
            
            # Extract intensity if available
            intensity = None
            if hasattr(self.las_data, 'intensity'):
                intensity = np.array(self.las_data.intensity[mask])
            
            # Extract normals if available
            normals = None
            if hasattr(self.las_data, 'NormalX'):
                normals = np.vstack((
                    self.las_data.NormalX[mask],
                    self.las_data.NormalY[mask],
                    self.las_data.NormalZ[mask]
                )).T
            
            extracted_classes = classifications[mask]
            
            # Save
            if self.radio_npz.isChecked():
                self._save_npz(output_file, xyz, colors, intensity, normals, extracted_classes)
            else:
                self._save_las(output_file, xyz, colors, intensity, normals, extracted_classes)
                
            # Show success
            class_names = [CLASSIFICATION_MAP.get(c, f"Class {c}") for c in selected_classes]
            QMessageBox.information(
                self,
                "Success",
                f"Extracted {len(xyz):,} points\n\n"
                f"Classes: {', '.join(class_names)}\n\n"
                f"Saved to: {output_file}"
            )
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to extract classes:\n{str(e)}")
            
    def _save_las(self, output_path, xyz, colors, intensity, normals, classifications):
        """Save to LAS file"""
        # Determine point format
        has_colors = colors is not None
        has_intensity = intensity is not None
        
        if has_colors and has_intensity:
            point_format = 3
        elif has_colors:
            point_format = 2
        elif has_intensity:
            point_format = 1
        else:
            point_format = 0
        
        # Create header
        header = laspy.LasHeader(point_format=point_format, version="1.2")
        header.offsets = self.las_data.header.offsets
        header.scales = self.las_data.header.scales
        
        # Create LAS data
        las = laspy.LasData(header)
        
        las.x = xyz[:, 0]
        las.y = xyz[:, 1]
        las.z = xyz[:, 2]
        las.classification = classifications.astype(np.uint8)
        
        if intensity is not None:
            las.intensity = intensity.astype(np.uint16)
        
        if colors is not None:
            if colors.max() <= 255:
                colors = (colors.astype(np.uint32) * 257).astype(np.uint16)
            las.red = colors[:, 0]
            las.green = colors[:, 1]
            las.blue = colors[:, 2]
        
        if normals is not None:
            try:
                las.add_extra_dim(laspy.ExtraBytesParams(name="NormalX", type=np.float32))
                las.add_extra_dim(laspy.ExtraBytesParams(name="NormalY", type=np.float32))
                las.add_extra_dim(laspy.ExtraBytesParams(name="NormalZ", type=np.float32))
                
                las.NormalX = normals[:, 0].astype(np.float32)
                las.NormalY = normals[:, 1].astype(np.float32)
                las.NormalZ = normals[:, 2].astype(np.float32)
            except:
                pass
        
        las.write(output_path)
        
    def _save_npz(self, output_path, xyz, colors, intensity, normals, classifications):
        """Save to NPZ file"""
        data = {
            'xyz': xyz,
            'classifications': classifications
        }
        
        if colors is not None:
            data['colors'] = colors
        
        if intensity is not None:
            data['intensity'] = intensity
        
        if normals is not None:
            data['normals'] = normals
        
        np.savez_compressed(output_path, **data)


if __name__ == "__main__":
    import sys
    
    app = QApplication(sys.argv)
    window = ClassificationExtractorGUI()
    window.show()
    sys.exit(app.exec())
