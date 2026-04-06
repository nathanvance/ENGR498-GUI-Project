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
#"C:\Users\henry\Downloads\law2-matched-filtered-classifier-powerline-flainet\law2_matched_filtered_-_classifier_-_powerline_flainet\wire_points.las"
#C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\ENGR-498-Project\Matlab_ExtractPowerLine\powerlineAerialLidarData.las
# matlab_path = r"C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\ENGR-498-Project\assets\scan_002\processed\filtered\cloud_filtered.las"
matlab_path = r"C:\Users\henry\Downloads\slt3_filtered.las"
#C:\Users\henry\Downloads\LAW2_matched_filtered.las"
#C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\ENGR-498-Project\Matlab_ExtractPowerLine\demo_extract_powerline.m
eng.addpath(r"C:\Users\henry\Downloads\3DLiDAR-main\3DLiDAR-main\ExtractPowerLine", 
            nargout=0)

#C:\Users\henry\Downloads\3DLiDAR-main\3DLiDAR-main\ExtractPowerLine\demo_extract_powerline.m

print("Running MATLAB wire extraction…")
# PL, poly = eng.demo_extract_powerline(matlab_path, nargout=2)

#original wire extraction output
PL, poly, ground_pts = eng.demo_extract_powerline(matlab_path, nargout=3)

# Save in NPZ format
PL_pts_np = np.array(PL)
np.savez("wires_points.npz", wires=PL_pts_np)
print("Saved wires points to wires_points.npz")

# Convert MATLAB array → NumPy -commented out
# 

# -----------------------------
# SAVE POLY INFO
# -----------------------------
import json

print("Saving wire info…")

poly_list = []
for i, entry in enumerate(poly):
    poly_list.append({
        "p": list(entry["p"][0]),        # polynomial coefficients
        "normr": float(entry["normr"]),
        "df": float(entry["df"]),
        "rsq": float(entry["rsq"]),
        "mu_mean": float(entry["mu_mean"]),
        "mu_std": float(entry["mu_std"])
    })

with open("wire_info.json", "w") as f:
    json.dump(poly_list, f, indent=2)

print("✓ Saved wire_info.json")

print("\n=== DEBUG: MATLAB poly structure ===")

# Print top-level keys
try:
    print("poly.keys():", poly.keys())
except:
    print("poly is not a dict. Type:", type(poly))

# Print full structure
print("\nFull poly structure:")
print(poly)

ground_pts_np = np.array(ground_pts)
np.savez("ground_points.npz", ground_points=ground_pts_np)
print("Saved ground points to ground_points.npz")

eng.quit()
print("MATLAB engine closed.")

#so, the poly variable is likely a MATLAB struct array, which doesn't directly translate to a Python dict.
#the ground points and wire points need to be save to the assets folder for the GUI to access them.


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