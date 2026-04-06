from pathlib import Path

import numpy as np
import laspy
import pyvista as pv
import json
from scipy.optimize import curve_fit
from scipy.spatial import cKDTree

################################################
####Still has buggy ground clearance line#######
################################################

from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel,
    QCheckBox, QScrollArea, QSizePolicy, QFrame, QSplitter
)
from PySide6.QtCore import Qt, Signal
from pyvistaqt import QtInteractor

from project_paths import (
    DEFAULT_GROUND_POINTS_NPZ_PATH,
    DEFAULT_LAS_PATH,
    DEFAULT_WIRES_NPZ_PATH,
    DEFAULT_WIRE_INFO_JSON_PATH,
)

LAS_PATH = DEFAULT_LAS_PATH

CLASS_NAME_MAP = {
    0: "Not classified",
    1: "Other",
    2: "Ground",
    3: "Vegetation",
    4: "Buildings",
    5: "Low noise",
    6: "High noise",
    7: "Vehicles",
    8: "Low voltage wire",
    9: "High voltage wire",
    10: "Railroad wire",
    11: "Low voltage tower",
    12: "High voltage tower",
    13: "Railroad tower",
    14: "Fences",
    15: "Insulator",
    16: "Guy wire",
    17: "Tension wire",
    18: "Cross-arms",
    19: "Pedestals"
}


def load_las_xyz_and_classes(path):
    las = laspy.read(path)
    xyz = np.vstack((las.x, las.y, las.z)).T
    classes = None
    try:
        # Try to get classification data
        classes = np.array(las.classification, dtype=np.int32)
        print(f"Loaded {len(classes)} points with classifications")
        unique_classes = np.unique(classes)
        print(f"Unique classes found: {unique_classes}")
    except Exception as e:
        print(f"Warning: Could not load classification data: {e}")
        classes = np.zeros(xyz.shape[0], dtype=np.int32)
    return xyz, classes


def generate_deterministic_colors(n, seed=0):
    rng = np.random.default_rng(seed)
    colors = [tuple(int(x) for x in rng.integers(30, 230, size=3)) for _ in range(n)]
    return colors


class SemanticViewer(QWidget):
    backRequested = Signal()

    # class-wide adjustable point size
    WIRE_POINT_SIZE = 2
    LAS_POINT_SIZE = 1

    def __init__(self, parent=None):
        super().__init__(parent)
        # UI setup
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # ========================
        #  Primary Splitter Layout
        # ========================
        # Left: 3D view
        # Right: all control panels
        self.splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(self.splitter)

        # ========================
        #  LEFT — LIDAR VIEWER
        # ========================
        viewer_container = QWidget()
        viewer_layout = QVBoxLayout()
        viewer_container.setLayout(viewer_layout)

        self.plotter = QtInteractor(self)
        # reduce viewer size (was ~900x700)
        self.plotter.setMinimumSize(650, 550)
        viewer_layout.addWidget(self.plotter)

        self.splitter.addWidget(viewer_container)

        # ========================
        #  RIGHT — SIDE PANELS
        # ========================
        right_panel = QWidget()
        right_layout = QHBoxLayout()
        right_layout.setContentsMargins(4, 4, 4, 4)
        right_panel.setLayout(right_layout)
        self.splitter.addWidget(right_panel)

        # initial splitter sizes (viewer smaller, side panels wider)
        self.splitter.setSizes([600, 760])   # tuned for your 1366x768

        # ------------------------
        # Classes panel
        # ------------------------
        classes_widget = QWidget()
        classes_layout = QVBoxLayout()
        classes_layout.setContentsMargins(4, 4, 4, 4)
        classes_widget.setLayout(classes_layout)

        classes_layout.addWidget(QLabel("Semantic Classes", alignment=Qt.AlignLeft))
        self.classes_scroll = QScrollArea()
        self.classes_scroll.setWidgetResizable(True)
        classes_content = QWidget()
        self.classes_list_layout = QVBoxLayout()
        self.classes_list_layout.setContentsMargins(0,0,0,0)
        classes_content.setLayout(self.classes_list_layout)
        self.classes_scroll.setWidget(classes_content)
        classes_layout.addWidget(self.classes_scroll, stretch=1)

        self.cb_toggle_las = QCheckBox("Show original LAS points (grey)")
        self.cb_toggle_las.setChecked(False)  # Start with grey points OFF
        # use toggled(bool) for a clean boolean, but _toggle_las_visible handles both
        self.cb_toggle_las.toggled.connect(self._toggle_las_visible)
        classes_layout.addWidget(self.cb_toggle_las)

        classes_layout.addStretch()
        right_layout.addWidget(classes_widget, stretch=1)

        # ------------------------
        # Wires panel
        # ------------------------
        wires_widget = QWidget()
        wires_layout = QVBoxLayout()
        wires_layout.setContentsMargins(4,4,4,4)
        wires_widget.setLayout(wires_layout)

        wires_layout.addWidget(QLabel("Wires", alignment=Qt.AlignLeft))
        self.wires_scroll = QScrollArea()
        self.wires_scroll.setWidgetResizable(True)
        wires_content = QWidget()
        self.wires_content_layout = QVBoxLayout()
        self.wires_content_layout.setContentsMargins(0, 0, 0, 0)
        wires_content.setLayout(self.wires_content_layout)
        self.wires_scroll.setWidget(wires_content)
        wires_layout.addWidget(self.wires_scroll, stretch=1)

        wires_layout.addStretch()
        right_layout.addWidget(wires_widget, stretch=1)

        # ------------------------
        # Wire Info panel
        # ------------------------
        info_widget = QWidget()
        info_layout = QVBoxLayout()
        info_layout.setContentsMargins(4,4,4,4)
        info_widget.setLayout(info_layout)

        info_title = QLabel("Wire Info")
        info_title.setStyleSheet("font-weight:bold; font-size:14px;")
        info_layout.addWidget(info_title)

        self.info_label = QLabel("Select a wire to see details.")
        self.info_label.setWordWrap(True)
        self.info_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        info_layout.addWidget(self.info_label, stretch=1)

        info_layout.addStretch()
        right_layout.addWidget(info_widget, stretch=1)

        # Storage
        self.las_actor = None
        self.class_actors = {}  # {class_id: actor}
        self.class_colors = {}  # {class_id: (r,g,b) tuple}
        self.wire_actors = []
        self.curve_actors = []
        self.wire_colors = []             # original int RGB tuples (0-255)
        self._actor_orig_colors = []      # normalized float colors (0-1) for restoration
        self.wire_widgets = []
        self.selected_wire_index = None
        self.clearance_actor = None

        # ground-related
        self.ground_points = None
        self.ground_kdtree = None
        self.ground_clearances = []  # list of (clearance, lowest_point, ground_point)
        self.catenary_params = []  # list of catenary parameters (a, b, c, t0, mean_xy, u_xy, mean_perp, perp_xy)

        self.xyz = None
        self.classes = None
        self.unique_classes = None
        self.class_checkboxes = {}  # {class_id: checkbox}

    # -----------------------
    # Actor utility helpers
    # -----------------------
    def _set_visibility(self, actor, visible: bool):
        """Robustly set visibility for a pyvista/VTK actor wrapper or pyvista actor object."""
        if actor is None:
            return
        try:
            # pyvista plotting.PolyDataActor wrapper
            if hasattr(actor, "actor") and hasattr(actor.actor, "SetVisibility"):
                actor.actor.SetVisibility(1 if visible else 0)
                return
            # pyvista Actor (may have SetVisibility)
            if hasattr(actor, "SetVisibility"):
                actor.SetVisibility(1 if visible else 0)
                return
            # vtk actor fallback
            if hasattr(actor, "SetVisibility"):
                actor.SetVisibility(1 if visible else 0)
                return
        except Exception:
            pass
        # best-effort: try attribute names
        try:
            if hasattr(actor, "visible"):
                actor.visible = visible
        except Exception:
            pass

    def _get_actor_color(self, actor):
        """Return (r,g,b) floats (0-1) if possible, else None."""
        try:
            if hasattr(actor, "actor") and hasattr(actor.actor, "GetProperty"):
                pr = actor.actor.GetProperty()
                col = pr.GetColor()
                # GetColor returns 3 floats 0-1
                return tuple(float(x) for x in col)
            if hasattr(actor, "GetProperty"):
                pr = actor.GetProperty()
                col = pr.GetColor()
                return tuple(float(x) for x in col)
            # pyvista wrapper might have .GetColor
            if hasattr(actor, "GetColor"):
                c = actor.GetColor()
                # could be 0-255 ints
                if isinstance(c[0], int) or c[0] > 1.0:
                    return (c[0]/255.0, c[1]/255.0, c[2]/255.0)
                return tuple(float(x) for x in c)
        except Exception:
            pass
        return None

    def _set_actor_color(self, actor, color_rgb):
        """
        color_rgb: either tuple floats 0-1 or ints 0-255
        """
        if actor is None:
            return
        # normalize ints -> floats
        if color_rgb is None:
            return
        try:
            if color_rgb[0] > 1.0:
                color = (color_rgb[0]/255.0, color_rgb[1]/255.0, color_rgb[2]/255.0)
            else:
                color = tuple(float(c) for c in color_rgb)
        except Exception:
            color = (1.0, 1.0, 0.0)

        try:
            if hasattr(actor, "actor") and hasattr(actor.actor, "GetProperty"):
                actor.actor.GetProperty().SetColor(*color)
                return
            if hasattr(actor, "GetProperty"):
                actor.GetProperty().SetColor(*color)
                return
            if hasattr(actor, "SetColor"):
                # pyvista SetColor accepts 0-255 or 0-1 depending; use 0-255 for that API
                actor.SetColor(tuple(int(c*255) for c in color))
                return
        except Exception:
            pass

    # -----------------------
    # LAS loading & rendering
    # -----------------------
    def load_las_file(self, path=LAS_PATH):
        self.xyz, self.classes = load_las_xyz_and_classes(path)
        self.unique_classes = np.unique(self.classes)
        print(f"Processing {len(self.unique_classes)} unique classes: {self.unique_classes}")
        
        self._render_class_list()
        self._render_class_actors()

    def _render_class_list(self):
        """Create UI checkboxes for each class found in the data"""
        # Clear existing widgets
        for i in reversed(range(self.classes_list_layout.count())):
            w = self.classes_list_layout.itemAt(i).widget()
            if w:
                w.setParent(None)
        
        self.class_checkboxes.clear()
        
        # Add title
        title = QLabel("Semantic Classes")
        title.setStyleSheet("font-weight:bold;")
        self.classes_list_layout.addWidget(title)
        
        # Generate colors for each unique class
        n_classes = len(self.unique_classes)
        colors = generate_deterministic_colors(n_classes, seed=42)
        
        # Create checkbox for each class that exists in the data
        for i, class_id in enumerate(sorted(self.unique_classes)):
            # Count points in this class
            count = np.sum(self.classes == class_id)
            percentage = (count / len(self.classes)) * 100
            
            # Get class name
            class_name = CLASS_NAME_MAP.get(class_id, f"Unknown ({class_id})")
            
            # Create checkbox with count info
            cb = QCheckBox(f"[{class_id}] {class_name} ({count:,} pts, {percentage:.1f}%)")
            cb.setChecked(True)  # Start with all classes visible
            cb.toggled.connect(lambda state, cid=class_id: self._toggle_class_visible(cid, state))
            
            self.classes_list_layout.addWidget(cb)
            self.class_checkboxes[class_id] = cb
            
            # Store color for this class
            self.class_colors[class_id] = colors[i]
        
        self.classes_list_layout.addStretch()

    def _render_class_actors(self):
        """Create separate actors for each classification class"""
        print("Rendering class actors...")
        
        # Clear any existing class actors
        for actor in self.class_actors.values():
            try:
                self.plotter.remove_actor(actor)
            except:
                pass
        self.class_actors.clear()
        
        # Create an actor for each class
        for class_id in self.unique_classes:
            mask = self.classes == class_id
            class_points = self.xyz[mask]
            
            if len(class_points) == 0:
                continue
            
            # Create point cloud for this class
            pdata = pv.PolyData(class_points)
            color = self.class_colors.get(class_id, (150, 150, 150))
            
            # Add to plotter
            actor = self.plotter.add_mesh(
                pdata, 
                color=color, 
                point_size=self.LAS_POINT_SIZE,
                render_points_as_spheres=False, 
                name=f"class_{class_id}"
            )
            
            self.class_actors[class_id] = actor
            print(f"  Created actor for class {class_id}: {len(class_points):,} points")
        
        # Create grey background actor (all points, initially hidden)
        pdata = pv.PolyData(self.xyz)
        dim_gray = (150, 150, 150)
        self.las_actor = self.plotter.add_mesh(
            pdata, color=dim_gray, point_size=self.LAS_POINT_SIZE,
            render_points_as_spheres=False, name="LAS_background"
        )
        # Hide it by default since we're showing colored classes
        self._set_visibility(self.las_actor, False)
        
        self.plotter.reset_camera()
        self.plotter.render()
        print("Class rendering complete!")

    def _toggle_class_visible(self, class_id, state):
        """Toggle visibility of a specific classification class"""
        if isinstance(state, bool):
            visible = state
        else:
            visible = (state == Qt.Checked)
        
        if class_id not in self.class_actors:
            return
        
        actor = self.class_actors[class_id]
        self._set_visibility(actor, visible)
        self.plotter.render()
        print(f"Class {class_id} visibility: {visible}")

    def _toggle_las_visible(self, state):
        """
        state may be int (stateChanged) or bool (toggled). Support both.
        """
        if isinstance(state, bool):
            visible = state
        else:
            visible = (state == Qt.Checked)
        if self.las_actor is None:
            return
        self._set_visibility(self.las_actor, visible)
        self.plotter.render()

    # -----------------------
    # Ground helpers
    # -----------------------
    def load_ground_points(self, path=DEFAULT_GROUND_POINTS_NPZ_PATH):
        try:
            data = np.load(Path(path), allow_pickle=True)
            # expecting either single array or keyed array 'ground_points'
            if "ground_points" in data.files:
                pts = np.asarray(data["ground_points"])  # (M,3)
            elif "xyz" in data.files:
                pts = np.asarray(data["xyz"])
            else:
                # pick first array
                pts = np.asarray(data[data.files[0]])
            self.ground_points = pts
            self.ground_kdtree = cKDTree(self.ground_points)
            print(f"Loaded {len(pts)} ground points.")
        except Exception as e:
            print("Failed to load ground points:", e)
            self.ground_points = None
            self.ground_kdtree = None

    def compute_ground_clearance(self, curve_points, catenary_params=None):
        """
        Returns (clearance_distance, lowest_point_on_curve, nearest_ground_point)
        
        Uses the middle point of the fitted curve to measure ground clearance.
        This ensures the measurement is taken at the center of the wire span.
        
        Args:
            curve_points: numpy array of curve points (N, 3)
            catenary_params: tuple (unused, kept for compatibility)
        """
        if curve_points is None or len(curve_points) == 0:
            return None, None, None
        if self.ground_kdtree is None:
            return None, None, None

        # Use the middle point of the curve (center of wire span)
        # This is guaranteed to be at the center regardless of wire orientation
        idx_middle = len(curve_points) // 2
        lowest = curve_points[idx_middle]
        
        # Find nearest ground point
        dist, gi = self.ground_kdtree.query(lowest)
        nearest_ground = self.ground_points[int(gi)]
        
        print(f"Ground clearance at middle point: z={lowest[2]:.3f}m, clearance={dist:.3f}m")
        
        return float(dist), lowest, nearest_ground

    # -----------------------
    # Wire loading & rendering
    # -----------------------
    def load_saved_wire_files(self,
                              points_npz_path=DEFAULT_WIRES_NPZ_PATH,
                              info_json_path=DEFAULT_WIRE_INFO_JSON_PATH,
                              ground_npz_path=DEFAULT_GROUND_POINTS_NPZ_PATH):
        # load wires
        npz = np.load(Path(points_npz_path), allow_pickle=True)

        # Extract MATLAB-style struct
        wires_struct = npz["wires"].item()

        # This is already a list of wire point arrays
        locations = [np.array(w, dtype=float) for w in wires_struct["Location"]]
        # load poly info
        with open(Path(info_json_path), "r") as f:
            poly = json.load(f)

        # load ground points & kdtree (optional)
        self.load_ground_points(ground_npz_path)

        PL = {
            "Location": locations,
            "Label": list(range(len(locations))),
            "Count": [len(x) for x in locations],
            "Ids": list(range(len(locations)))
        }
        self.load_wires(PL, poly)

    def load_wires(self, PL: dict, poly: list):
        # Clear previous UI and hide old actors (do NOT remove actors — that caused toggle issues)
        self._clear_wires()
        locations = PL.get("Location", [])
        n_wires = len(locations)
        self.wire_colors = generate_deterministic_colors(n_wires, seed=123)
        self._actor_orig_colors = [None] * n_wires
        self.ground_clearances = [None] * n_wires
        self.catenary_params = []  # Reset catenary params list

        for i in range(n_wires):
            pts = np.array(locations[i])
            pdata = pv.PolyData(pts)
            color = self.wire_colors[i]
            actor = self.plotter.add_mesh(
                pdata, color=color, point_size=self.WIRE_POINT_SIZE,
                render_points_as_spheres=True, name=f"wire_points_{i}"
            )
            # record actor and its exact original color
            self.wire_actors.append(actor)
            # capture normalized color for restoration
            col = self._get_actor_color(actor)
            if col is None:
                # fallback convert int colors
                col = (color[0]/255.0, color[1]/255.0, color[2]/255.0)
            self._actor_orig_colors[i] = col

            # Curve overlay (fit catenary in Python)
            curve_actor = None
            curve_points = None
            catenary_params = None
            try:
                # NOW RETURNS BOTH POINTS AND PARAMS
                curve_points, catenary_params = self.fit_catenary_wire(pts)
                if curve_points is not None and curve_points.size > 0:
                    spline = pv.Spline(curve_points, len(curve_points))
                    curve_actor = self.plotter.add_mesh(
                        spline, color=color, line_width=3, name=f"wire_curve_{i}"
                    )
            except Exception as e:
                print("Error creating curve for wire", i, e)

            self.curve_actors.append(curve_actor)
            self.catenary_params.append(catenary_params)  # Store params for this wire

            # compute clearance and store - NOW PASS PARAMS
            clearance_info = (None, None, None)
            if curve_points is not None and curve_points.size > 0:
                clearance_info = self.compute_ground_clearance(curve_points, catenary_params)
            self.ground_clearances[i] = clearance_info

            self._create_wire_ui(i, poly[i] if i < len(poly) else None)

        self.plotter.render()

    def fit_catenary_wire(self, pts, n_samples=300):
        """
        Robust catenary fit for a 3D wire point cloud.
        Method: PCA in XY to compute along-wire coordinate t, fit z(t) to catenary.
        
        Returns:
            tuple: (curve_points, catenary_params) where catenary_params is 
                   (a, b, c, t0, mean_xy, u_xy, mean_perp, perp_xy) or None
        """
        try:
            pts = np.asarray(pts, dtype=float)
            if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 4:
                return np.array([]), None

            xy = pts[:, :2]
            z = pts[:, 2]
            mean_xy = xy.mean(axis=0)
            xy_centered = xy - mean_xy
            U, S, Vt = np.linalg.svd(xy_centered, full_matrices=False)
            u_xy = Vt[0]
            perp_xy = np.array([-u_xy[1], u_xy[0]])

            t = xy_centered.dot(u_xy)
            order = np.argsort(t)
            t_sorted = t[order]
            z_sorted = z[order]

            t0 = t_sorted.min()
            t_norm = t_sorted - t0

            def catenary(x, a, b, c):
                return a * np.cosh((x - b) / (a + 1e-12)) + c

            a0 = max((z_sorted.max() - z_sorted.min()) / 2.0, 0.1)
            b0 = t_norm.mean()
            c0 = z_sorted.min()
            p0 = [a0, b0, c0]

            t_range = t_norm.max() - t_norm.min()
            lower = [1e-6, -t_range - 1.0, -1e6]
            upper = [1e6, t_range * 2 + 1.0, 1e6]

            try:
                popt, pcov = curve_fit(
                    catenary, t_norm, z_sorted, p0=p0,
                    bounds=(lower, upper), maxfev=20000
                )
            except Exception:
                # fallback to linear interpolation along axis
                x_fit = np.linspace(t_sorted.min(), t_sorted.max(), n_samples)
                z_fit = np.interp(x_fit, t_sorted, z_sorted)
                perp_coords = xy_centered.dot(perp_xy)
                mean_perp = float(np.mean(perp_coords))
                xy_fit = mean_xy + np.outer(x_fit, u_xy) + np.outer(np.ones_like(x_fit) * mean_perp, perp_xy)
                pts_curve = np.column_stack((xy_fit, z_fit))
                return pts_curve, None

            a, b, c = popt
            t_fit_norm = np.linspace(t_norm.min(), t_norm.max(), n_samples)
            z_fit = catenary(t_fit_norm, a, b, c)
            t_fit = t_fit_norm + t0

            perp_coords = xy_centered.dot(perp_xy)
            mean_perp = float(np.mean(perp_coords))
            xy_fit = mean_xy + np.outer(t_fit, u_xy) + np.outer(np.ones_like(t_fit) * mean_perp, perp_xy)

            pts_curve = np.column_stack((xy_fit, z_fit))
            
            # Store parameters for later use in clearance calculation
            catenary_params = (a, b, c, t0, mean_xy, u_xy, mean_perp, perp_xy)
            
            return pts_curve, catenary_params

        except Exception as e:
            print("fit_catenary_wire error:", e)
            return np.array([]), None

    # -----------------------
    # Wire UI & selection
    # -----------------------
    def _create_wire_ui(self, idx, poly_entry):
        header_btn = QPushButton(f"▶ Wire {idx+1}")
        header_btn.setCheckable(True)
        header_btn.setChecked(False)

        controls = QWidget()
        controls_layout = QVBoxLayout()
        controls_layout.setContentsMargins(8, 0, 0, 0)
        controls.setLayout(controls_layout)
        controls.setVisible(False)

        cb_wire = QCheckBox("Set wire visible")
        cb_wire.setChecked(True)
        # use toggled(bool) so handler gets bool, but handler accepts both
        cb_wire.toggled.connect(lambda state, i=idx: self._toggle_wire_visible(i, state))
        controls_layout.addWidget(cb_wire)

        cb_curve = QCheckBox("Set line overlay visible")
        cb_curve.setChecked(True)
        cb_curve.toggled.connect(lambda state, i=idx: self._toggle_curve_visible(i, state))
        controls_layout.addWidget(cb_curve)

        def on_header_toggled(checked, i=idx):
            header_btn.setText(("▼ " if checked else "▶ ") + f"Wire {i+1}")
            controls.setVisible(checked)
            if checked:
                self._select_wire(i)
            else:
                if self.selected_wire_index == i:
                    self._clear_selection()

        header_btn.toggled.connect(on_header_toggled)

        container = QWidget()
        container_layout = QVBoxLayout()
        container_layout.setContentsMargins(0,0,0,0)
        container.setLayout(container_layout)
        container_layout.addWidget(header_btn)
        container_layout.addWidget(controls)

        self.wires_content_layout.addWidget(container)
        # store the UI widgets and poly entry
        self.wire_widgets.append({
            "container": container, "header": header_btn, "controls": controls,
            "cb_wire": cb_wire, "cb_curve": cb_curve, "poly": poly_entry
        })

    def _toggle_wire_visible(self, idx, state):
        if idx < 0 or idx >= len(self.wire_actors):
            return
        # accept bool or int
        if isinstance(state, bool):
            visible = state
        else:
            visible = (state == Qt.Checked)
        actor = self.wire_actors[idx]
        self._set_visibility(actor, visible)
        self.plotter.render()

    def _toggle_curve_visible(self, idx, state):
        if idx < 0 or idx >= len(self.curve_actors):
            return
        if isinstance(state, bool):
            visible = state
        else:
            visible = (state == Qt.Checked)
        actor = self.curve_actors[idx]
        if actor is None:
            return
        self._set_visibility(actor, visible)
        self.plotter.render()

    def _draw_clearance_line(self, idx):
        """
        Draw a dashed vertical line from lowest point on catenary
        to nearest ground point for wire `idx`.
        Hides previous clearance line if present.
        """

        # Hide previous clearance line
        if self.clearance_actor is not None:
            self.clearance_actor.SetVisibility(0)

        if idx < 0 or idx >= len(self.ground_clearances):
            return

        dist, low_pt, grd_pt = self.ground_clearances[idx]
        if low_pt is None or grd_pt is None:
            return

        # Build line segment
        line = pv.Line(low_pt, grd_pt)

        # Add line – in your PyVista version this RETURNS A VTK ACTOR
        vtk_actor = self.plotter.add_mesh(
            line,
            color="yellow",
            line_width=3,
            name="clearance_line"
        )

        # configure dashed style
        prop = vtk_actor.GetProperty()
        prop.SetLineStipplePattern(0x00FF)
        prop.SetLineStippleRepeatFactor(2)
        prop.SetLineStipple(True)

        vtk_actor.SetVisibility(1)

        # store for toggling/hiding
        self.clearance_actor = vtk_actor

        self.plotter.render()


    def _select_wire(self, idx):
        # restore previous selection color/size
        if self.selected_wire_index is not None and self.selected_wire_index != idx:
            self._restore_wire_color(self.selected_wire_index)

        # apply highlight to this wire
        self._highlight_wire(idx)
        self.selected_wire_index = idx

        poly_entry = self.wire_widgets[idx]["poly"]
        # show poly info + clearance
        info_text = self._format_poly_info(idx, poly_entry)
        # append clearance if available
        clear_info = self.ground_clearances[idx] if idx < len(self.ground_clearances) else None
        if clear_info is not None and clear_info[0] is not None:
            info_text += f"\nGround clearance (lowest point): {clear_info[0]:.3f} m"
        self.info_label.setText(info_text)
        #draws clearance line
        self._draw_clearance_line(idx)
        self.plotter.render()

    def _clear_selection(self):
        if self.selected_wire_index is not None:
            self._restore_wire_color(self.selected_wire_index)
        self.selected_wire_index = None
        self.info_label.setText("Select a wire to see details.")
        if self.clearance_actor is not None:
            self.clearance_actor.SetVisibility(0)
        self.plotter.render()

    def _highlight_wire(self, idx):
        if idx < 0 or idx >= len(self.wire_actors):
            return
        actor = self.wire_actors[idx]
        # store original color if not stored
        if idx < len(self._actor_orig_colors) and self._actor_orig_colors[idx] is None:
            self._actor_orig_colors[idx] = self._get_actor_color(actor) or (self.wire_colors[idx][0]/255.0, self.wire_colors[idx][1]/255.0, self.wire_colors[idx][2]/255.0)

        # set highlight color (yellow) and bump size
        self._set_actor_color(actor, (1.0, 1.0, 0.0))
        try:
            if hasattr(actor, "actor") and hasattr(actor.actor.GetProperty(), "SetPointSize"):
                actor.actor.GetProperty().SetPointSize(max(self.WIRE_POINT_SIZE * 1.5, 6))
        except Exception:
            pass

    def _restore_wire_color(self, idx):
        if idx < 0 or idx >= len(self.wire_actors):
            return
        actor = self.wire_actors[idx]
        # restore saved normalized color if available, else fallback to self.wire_colors
        if idx < len(self._actor_orig_colors) and self._actor_orig_colors[idx] is not None:
            orig_col = self._actor_orig_colors[idx]
            self._set_actor_color(actor, orig_col)
        else:
            self._set_actor_color(actor, self.wire_colors[idx])
        try:
            if hasattr(actor, "actor") and hasattr(actor.actor.GetProperty(), "SetPointSize"):
                actor.actor.GetProperty().SetPointSize(self.WIRE_POINT_SIZE)
        except Exception:
            pass

    # -----------------------
    # Utilities
    # -----------------------
    def _format_poly_info(self, idx, poly_entry):
        if poly_entry is None:
            return "No poly info available for this wire."
        p = poly_entry.get("p", [])
        normr = poly_entry.get("normr", None)
        df = poly_entry.get("df", None)
        rsq = poly_entry.get("rsq", None)
        if "mu_mean" in poly_entry and "mu_std" in poly_entry:
            mu_mean, mu_std = poly_entry["mu_mean"], poly_entry["mu_std"]
        elif "mu" in poly_entry:
            mu_mean, mu_std = poly_entry["mu"]
        else:
            mu_mean = mu_std = None

        txt = f"=== Wire {idx+1} ===\n"
        try:
            txt += f"p = [{p[0]:.6f}  {p[1]:.6f}  {p[2]:.6f}]\n"
        except Exception:
            txt += "p = [N/A]\n"
        if normr and df and rsq:
            txt += f"S.normr = {normr:.6f} | S.df = {int(df)} | S.rsquared = {rsq:.4f}\n"
        elif normr and df:
            txt += f"S.normr = {normr:.6f} | S.df = {int(df)}\n"
        if mu_mean and mu_std:
            txt += f"mu = [mean={mu_mean:.6f}, std={mu_std:.6f}]\n"
        return txt

    def _clear_wires(self):
        # Hide existing actors instead of removing them. This preserves actor objects
        # so toggles can reliably SetVisibility(1/0) later without losing underlying VTK object.
        for a in self.wire_actors:
            try:
                self._set_visibility(a, False)
            except Exception:
                pass
        for a in self.curve_actors:
            if a:
                try:
                    self._set_visibility(a, False)
                except Exception:
                    pass
        # Remove UI widgets
        for i in reversed(range(self.wires_content_layout.count())):
            w = self.wires_content_layout.itemAt(i).widget()
            if w:
                w.setParent(None)
        # Now clear internal lists (actor objects remain hidden in the scene)
        self.wire_actors = []
        self.curve_actors = []
        self.wire_widgets = []
        self.wire_colors = []
        self._actor_orig_colors = []
        self.selected_wire_index = None
        self.ground_clearances = []
        self.info_label.setText("Select a wire to see details.")
        self.plotter.render()

    def initialize_viewer(self):
        self.load_las_file()
        # attempt to load saved wire & ground files (if they exist)
        try:
            self.load_saved_wire_files()
        except Exception as e:
            print("Failed to load saved wire/ground files:", e)
