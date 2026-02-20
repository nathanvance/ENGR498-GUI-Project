# project_state.py
from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import numpy as np


@dataclass
class ProjectState:
    """
    Central shared state for the LiDAR processing pipeline.
    This object should be passed (by reference) to all views.
    """

    # -------------------------
    # Raw input
    # -------------------------
    las_path: Optional[str] = None
    raw_points: Optional[np.ndarray] = None        # (N, 3) or (N, 4)

    # -------------------------
    # Preprocessing
    # -------------------------
    preprocessed_points: Optional[np.ndarray] = None
    preprocessing_params: Dict[str, Any] = field(default_factory=dict)

    # -------------------------
    # FLAI semantic segmentation
    # -------------------------
    flai_uploaded: bool = False
    flai_job_id: Optional[str] = None
    flai_semantic_points: Optional[np.ndarray] = None
    flai_labels: Optional[np.ndarray] = None        # (N,)

    # -------------------------
    # Wire extraction & modeling
    # -------------------------
    wire_points: Optional[np.ndarray] = None        # extracted wire points
    catenary_models: Optional[list] = None          # per-wire catenary fits
    ground_points: Optional[np.ndarray] = None
    ground_clearance: Optional[Dict[str, float]] = None

    # -------------------------
    # Fusion (image → point cloud)
    # -------------------------
    fused_points: Optional[np.ndarray] = None
    fused_labels: Optional[np.ndarray] = None

    # -------------------------
    # Final visualization
    # -------------------------
    final_points: Optional[np.ndarray] = None

    # -------------------------
    # Utility
    # -------------------------
    def reset_to_raw(self):
        """Revert all downstream results back to raw input."""
        self.preprocessed_points = None
        self.preprocessing_params.clear()

        self.flai_uploaded = False
        self.flai_job_id = None
        self.flai_semantic_points = None
        self.flai_labels = None

        self.wire_points = None
        self.catenary_models = None
        self.ground_points = None
        self.ground_clearance = None

        self.fused_points = None
        self.fused_labels = None
        self.final_points = None

    def has_raw_data(self) -> bool:
        return self.raw_points is not None or self.las_path is not None

    def has_preprocessed_data(self) -> bool:
        return self.preprocessed_points is not None

    def has_flai_results(self) -> bool:
        return self.flai_semantic_points is not None

    def has_wire_results(self) -> bool:
        return self.wire_points is not None and self.catenary_models is not None

    #def initialize_project_state(self, file_path): this can be a constructor for initializing the state when a new file is loaded,
    # or it can be a method that resets the state and loads new data.
        #self.las_path = file_path
        #self.raw_points = load_las_points(file_path)  # hypothetical function to load raw points from LAS
        #self.

    #for use in uploading to FLAI
    #def write_to_las(self):

    #def format_input_for_matlab(self):

    

