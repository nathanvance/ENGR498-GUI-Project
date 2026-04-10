from __future__ import annotations

from PySide6.QtGui import QColor, QPalette


THEME = {
    "bg": "#0b1220",
    "bg_elevated": "#111a2e",
    "pane": "#162133",
    "pane_raised": "#1b2940",
    "pane_alt": "#22324a",
    "input": "#0f1728",
    "border": "#2a3a54",
    "border_strong": "#3a4f70",
    "text": "#e5edf7",
    "muted": "#93a4bf",
    "accent": "#3b82f6",
    "accent_hover": "#60a5fa",
    "selection": "#1d4ed8",
    "success": "#22c55e",
    "warning": "#f59e0b",
    "danger": "#ef4444",
    "info": "#06b6d4",
    "radius_sm": "6px",
    "radius_md": "10px",
    "radius_lg": "14px",
}


def build_palette() -> QPalette:
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(THEME["bg"]))
    palette.setColor(QPalette.WindowText, QColor(THEME["text"]))
    palette.setColor(QPalette.Base, QColor(THEME["input"]))
    palette.setColor(QPalette.AlternateBase, QColor(THEME["pane"]))
    palette.setColor(QPalette.ToolTipBase, QColor(THEME["pane_raised"]))
    palette.setColor(QPalette.ToolTipText, QColor(THEME["text"]))
    palette.setColor(QPalette.Text, QColor(THEME["text"]))
    palette.setColor(QPalette.Button, QColor(THEME["pane"]))
    palette.setColor(QPalette.ButtonText, QColor(THEME["text"]))
    palette.setColor(QPalette.BrightText, QColor("#ffffff"))
    palette.setColor(QPalette.Link, QColor(THEME["accent"]))
    palette.setColor(QPalette.Highlight, QColor(THEME["selection"]))
    palette.setColor(QPalette.HighlightedText, QColor("#ffffff"))
    palette.setColor(QPalette.PlaceholderText, QColor(THEME["muted"]))
    return palette


def button_style(background: str, hover: str, *, text: str = "#ffffff", disabled_bg: str = "#475569") -> str:
    return f"""
        QPushButton {{
            background-color: {background};
            color: {text};
            border: 1px solid transparent;
            padding: 10px 18px;
            border-radius: {THEME['radius_sm']};
            font-weight: bold;
        }}
        QPushButton:hover:enabled {{
            background-color: {hover};
        }}
        QPushButton:disabled {{
            background-color: {disabled_bg};
            color: {THEME['muted']};
            border-color: {THEME['border']};
        }}
    """


def subtle_button_style() -> str:
    return f"""
        QPushButton {{
            background-color: {THEME['pane_alt']};
            color: {THEME['text']};
            border: 1px solid {THEME['border']};
            padding: 8px 12px;
            border-radius: {THEME['radius_sm']};
            font-weight: 600;
        }}
        QPushButton:hover {{
            background-color: {THEME['pane_raised']};
            border-color: {THEME['border_strong']};
        }}
    """


def notification_panel_style() -> str:
    return f"""
        QWidget {{
            background-color: {THEME['pane']};
            border: 1px solid {THEME['border']};
            border-radius: {THEME['radius_md']};
        }}
    """


def notification_item_style() -> str:
    return f"""
        QFrame {{
            background-color: {THEME['pane_raised']};
            border: 1px solid {THEME['border']};
            border-radius: {THEME['radius_sm']};
            padding: 8px;
            margin: 2px;
        }}
    """


def build_stylesheet() -> str:
    t = THEME
    return f"""
        QWidget {{
            background-color: {t['bg']};
            color: {t['text']};
            font-family: 'Segoe UI', Arial, sans-serif;
        }}
        QMainWindow, QDialog {{
            background-color: {t['bg']};
        }}
        QLabel {{
            color: {t['text']};
            background: transparent;
        }}
        QToolBar {{
            background-color: {t['pane']};
            border: none;
            border-bottom: 1px solid {t['border']};
            spacing: 8px;
            padding: 8px;
        }}
        QToolBar QToolButton {{
            background-color: {t['pane_alt']};
            color: {t['text']};
            border: 1px solid {t['border']};
            border-radius: {t['radius_sm']};
            padding: 8px 12px;
            font-weight: 600;
        }}
        QToolBar QToolButton:hover {{
            background-color: {t['pane_raised']};
            border-color: {t['border_strong']};
        }}
        QStackedWidget {{
            background-color: {t['bg']};
        }}
        QFrame[card="true"], QGroupBox, QMenu, QMessageBox, QFileDialog {{
            background-color: {t['pane']};
            border: 1px solid {t['border']};
            border-radius: {t['radius_md']};
        }}
        QGroupBox {{
            margin-top: 14px;
            padding-top: 14px;
            font-weight: 600;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 6px;
            color: {t['text']};
        }}
        QScrollArea, QScrollArea > QWidget > QWidget {{
            background-color: transparent;
        }}
        QLineEdit, QTextEdit, QPlainTextEdit, QComboBox, QSpinBox, QDoubleSpinBox, QListWidget, QTreeWidget, QTableWidget {{
            background-color: {t['input']};
            color: {t['text']};
            border: 1px solid {t['border']};
            border-radius: {t['radius_md']};
            selection-background-color: {t['selection']};
            selection-color: #ffffff;
        }}
        QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
            min-height: 18px;
            padding: 7px 9px;
        }}
        QComboBox::drop-down {{
            border: none;
            width: 20px;
        }}
        QAbstractItemView {{
            background-color: {t['pane']};
            color: {t['text']};
            border: 1px solid {t['border']};
            selection-background-color: {t['selection']};
        }}
        QHeaderView::section {{
            background-color: {t['pane_raised']};
            color: {t['text']};
            padding: 10px;
            border: none;
            border-bottom: 1px solid {t['border_strong']};
            font-weight: bold;
        }}
        QTableCornerButton::section {{
            background-color: {t['pane_raised']};
            border: none;
            border-bottom: 1px solid {t['border_strong']};
        }}
        QTextEdit[log="true"], QPlainTextEdit[log="true"] {{
            background-color: #09111f;
            color: #dbe6f5;
            border: 1px solid {t['border']};
            border-radius: {t['radius_md']};
            padding: 8px;
            font-family: Consolas, 'Courier New', monospace;
            font-size: 11px;
        }}
        QPushButton {{
            background-color: {t['pane_alt']};
            color: {t['text']};
            border: 1px solid {t['border']};
            border-radius: {t['radius_sm']};
            padding: 8px 12px;
            font-weight: 600;
        }}
        QPushButton:hover:enabled {{
            background-color: {t['pane_raised']};
            border-color: {t['border_strong']};
        }}
        QPushButton:disabled {{
            color: {t['muted']};
            background-color: #263243;
        }}
        QCheckBox {{
            color: {t['text']};
            spacing: 8px;
        }}
        QCheckBox::indicator {{
            width: 16px;
            height: 16px;
            border: 2px solid {t['border_strong']};
            border-radius: 3px;
            background: {t['input']};
        }}
        QCheckBox::indicator:checked {{
            background: {t['accent']};
            border-color: {t['accent']};
        }}
        QMenu::item:selected {{
            background-color: {t['selection']};
        }}
        QScrollBar:vertical {{
            background: {t['bg']};
            width: 12px;
            margin: 2px;
        }}
        QScrollBar::handle:vertical {{
            background: {t['pane_alt']};
            min-height: 24px;
            border-radius: 6px;
        }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical,
        QScrollBar:horizontal, QScrollBar::handle:horizontal,
        QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal,
        QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
            background: none;
            border: none;
        }}
    """
