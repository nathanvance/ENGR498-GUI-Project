# testMatlab.py
import matlab.engine
import numpy as np
import json
import sys
# from PySide6.QtWidgets import QApplication




# -----------------------------
# 1. RUN MATLAB EXTRACTION
# -----------------------------
eng = matlab.engine.start_matlab()

matlab_path = "powerlineAerialLidarData.las"

eng.addpath(r"C:\Users\henry\Downloads\3DLiDAR-main\3DLiDAR-main\ExtractPowerLine", 
            nargout=0)

print("Running MATLAB wire extraction…")
PL, poly, ground_pts = eng.demo_extract_powerline(matlab_path, nargout=3)

# Convert MATLAB array → NumPy
# ground_pts_np = np.array(ground_pts)

# # Save in NPZ format
# np.savez("ground_points.npz", ground_points=ground_pts_np)

# print("Saved ground points to ground_points.npz")


# -----------------------------
# SAVE POLY INFO
# -----------------------------
# import json

# print("Saving wire info…")

# poly_list = []
# for i, entry in enumerate(poly):
#     poly_list.append({
#         "p": list(entry["p"][0]),        # polynomial coefficients
#         "normr": float(entry["normr"]),
#         "df": float(entry["df"]),
#         "rsq": float(entry["rsq"]),
#         "mu_mean": float(entry["mu_mean"]),
#         "mu_std": float(entry["mu_std"])
#     })

# with open("wire_info.json", "w") as f:
#     json.dump(poly_list, f, indent=2)

# print("✓ Saved wire_info.json")

# print("\n=== DEBUG: MATLAB poly structure ===")

# # Print top-level keys
# try:
#     print("poly.keys():", poly.keys())
# except:
#     print("poly is not a dict. Type:", type(poly))

# # Print full structure
# print("\nFull poly structure:")
# print(poly)

# # If poly is an array of structs, inspect first element
# try:
#     print("\npoly[0] =", poly[0])
# except Exception as e:
#     print("poly[0] error:", e)

# # Try listing fields if it's a MATLAB struct
# try:
#     print("\npoly fields:", poly._fieldnames)
# except:
#     print("poly has no _fieldnames attribute")