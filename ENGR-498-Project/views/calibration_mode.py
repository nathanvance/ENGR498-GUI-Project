from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from theme import THEME, button_style, subtle_button_style


class CalibrationModeView(QWidget):
    """Standalone mode for direct visual LiDAR calibration workflow."""

    runCalibrationRequested = Signal(str, str, str)  # dataset_path, run_name, stop_after
    checkRequirementsRequested = Signal()
    switchToPostProcessingRequested = Signal()

    def __init__(self, default_output_root: Path, parent=None):
        super().__init__(parent)
        self.default_output_root = Path(default_output_root)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        self.setLayout(layout)

        self.setObjectName("calibrationModeView")

        title = QLabel("Calibration Mode")
        title.setStyleSheet(f"font-size: 22px; font-weight: bold; color: {THEME['text']};")
        layout.addWidget(title)

        summary = QLabel(
            "Use this mode to run the direct visual LiDAR calibration workflow in Docker. "
            "Calibration is a prerequisite for post-processing Fusion because Fusion consumes the completed calibration output."
        )
        summary.setWordWrap(True)
        summary.setStyleSheet(f"color: {THEME['muted']};")
        layout.addWidget(summary)

        requirements = QFrame()
        requirements.setProperty("card", True)
        requirements.setStyleSheet(
            f"QFrame {{ background-color: {THEME['pane_raised']}; border: 1px solid {THEME['warning']}; border-radius: {THEME['radius_md']}; }}"
        )
        requirements_layout = QVBoxLayout()
        requirements_layout.setContentsMargins(12, 10, 12, 10)
        requirements.setLayout(requirements_layout)
        requirements_label = QLabel(
            "<b>Calibration Requirements</b><br>"
            "Calibration requires a real GPU-backed OpenGL path through WSLg inside the Docker runtime.<br>"
            "Software rendering is not a supported calibration path on Windows.<br>"
            "Use <b>Check Requirements</b> before running calibration on a new machine."
        )
        requirements_label.setWordWrap(True)
        requirements_layout.addWidget(requirements_label)
        layout.addWidget(requirements)

        notes = QFrame()
        notes.setProperty("card", True)
        notes.setStyleSheet(
            f"QFrame {{ background-color: {THEME['pane']}; border: 1px solid {THEME['border']}; border-radius: {THEME['radius_md']}; }}"
        )
        notes_layout = QVBoxLayout()
        notes_layout.setContentsMargins(12, 10, 12, 10)
        notes.setLayout(notes_layout)
        notes_label = QLabel(
            "<b>What calibration mode does</b><br>"
            "- replays the calibration bag(s)<br>"
            "- republishes <code>/camera/image_raw</code> for the calibration tool<br>"
            "- extracts camera intrinsics from <code>/camera/camera_info</code><br>"
            "- runs direct visual LiDAR calibration and saves <code>calib.json</code><br>"
            "- produces the extrinsic LiDAR-camera transform used later by Fusion"
        )
        notes_label.setWordWrap(True)
        notes_layout.addWidget(notes_label)
        layout.addWidget(notes)

        dataset_row = QHBoxLayout()
        self.dataset_edit = QLineEdit()
        self.dataset_edit.setPlaceholderText("Select the calibration dataset folder that contains one or more .bag files")
        dataset_row.addWidget(self.dataset_edit, stretch=1)
        browse_btn = QPushButton("Browse Dataset")
        browse_btn.setStyleSheet(button_style(THEME["accent"], THEME["accent_hover"]))
        browse_btn.clicked.connect(self._browse_dataset)
        dataset_row.addWidget(browse_btn)
        layout.addLayout(dataset_row)

        run_row = QHBoxLayout()
        self.run_name_edit = QLineEdit()
        self.run_name_edit.setPlaceholderText("Enter a calibration run name, e.g. field_cal_01")
        run_row.addWidget(self.run_name_edit, stretch=1)
        defaults_btn = QPushButton("Use Timestamp Name")
        defaults_btn.setStyleSheet(subtle_button_style())
        defaults_btn.clicked.connect(self._apply_default_run_name)
        run_row.addWidget(defaults_btn)
        layout.addLayout(run_row)

        buttons_row = QHBoxLayout()
        check_btn = QPushButton("Check Requirements")
        check_btn.setStyleSheet(button_style(THEME["info"], "#0891b2"))
        check_btn.clicked.connect(self.checkRequirementsRequested.emit)
        buttons_row.addWidget(check_btn)

        run_full_btn = QPushButton("Run Calibration")
        run_full_btn.setStyleSheet(button_style("#7c3aed", "#8b5cf6"))
        run_full_btn.clicked.connect(lambda: self._emit_run(""))
        buttons_row.addWidget(run_full_btn)

        preprocess_btn = QPushButton("Preprocess Only")
        preprocess_btn.setStyleSheet(button_style(THEME["warning"], "#fbbf24", text="#111827"))
        preprocess_btn.clicked.connect(lambda: self._emit_run("preprocess"))
        buttons_row.addWidget(preprocess_btn)

        switch_btn = QPushButton("Go To Post-Processing")
        switch_btn.setStyleSheet(button_style(THEME["success"], "#4ade80"))
        switch_btn.clicked.connect(self.switchToPostProcessingRequested.emit)
        buttons_row.addWidget(switch_btn)
        buttons_row.addStretch()
        layout.addLayout(buttons_row)

        output_note = QLabel(
            f"Calibration outputs are written under:<br><code>{self.default_output_root.as_posix()}</code>"
        )
        output_note.setWordWrap(True)
        output_note.setStyleSheet(f"color: {THEME['muted']};")
        layout.addWidget(output_note)

        self.requirements_status_label = QLabel("Requirements check not run yet.")
        self.requirements_status_label.setWordWrap(True)
        self.requirements_status_label.setStyleSheet(
            f"color: {THEME['text']}; background-color: {THEME['pane_raised']}; border: 1px solid {THEME['border']}; border-radius: {THEME['radius_sm']}; padding: 8px;"
        )
        layout.addWidget(self.requirements_status_label)

        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet(f"color: {THEME['muted']}; padding-top: 6px;")
        layout.addWidget(self.status_label)

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMinimumHeight(280)
        self.log_output.setProperty("log", True)
        layout.addWidget(self.log_output, stretch=1)

    def _browse_dataset(self):
        chosen = QFileDialog.getExistingDirectory(self, "Select calibration dataset folder", "")
        if chosen:
            self.dataset_edit.setText(chosen)

    def _apply_default_run_name(self):
        from datetime import datetime

        self.run_name_edit.setText(datetime.now().strftime("calibration_%Y%m%d_%H%M%S"))

    def _emit_run(self, stop_after: str):
        dataset = self.dataset_edit.text().strip()
        run_name = self.run_name_edit.text().strip()
        if not dataset:
            QMessageBox.warning(self, "Calibration Mode", "Select a calibration dataset folder first.")
            return
        if not run_name:
            QMessageBox.warning(self, "Calibration Mode", "Enter a run name first.")
            return
        self.runCalibrationRequested.emit(dataset, run_name, stop_after)

    def append_log(self, line: str):
        self.log_output.append(line)

    def set_status(self, message: str):
        self.status_label.setText(message)

    def set_requirements_status(self, message: str, *, ok: bool | None = None):
        self.requirements_status_label.setText(message)
        if ok is True:
            style = (
                f"color: {THEME['text']}; background-color: {THEME['pane_raised']}; border: 1px solid {THEME['success']}; "
                "border-radius: 6px; padding: 8px;"
            )
        elif ok is False:
            style = (
                f"color: {THEME['text']}; background-color: {THEME['pane_raised']}; border: 1px solid {THEME['danger']}; "
                "border-radius: 6px; padding: 8px;"
            )
        else:
            style = (
                f"color: {THEME['text']}; background-color: {THEME['pane_raised']}; border: 1px solid {THEME['border']}; "
                "border-radius: 6px; padding: 8px;"
            )
        self.requirements_status_label.setStyleSheet(style)
