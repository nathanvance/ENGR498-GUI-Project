import laspy
import sys
import numpy as np

good_path = r"C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\ENGR-498-Project\Matlab_ExtractPowerLine\powerlineAerialLidarData.las"
bad_path  = r"C:\Users\henry\Downloads\movingtest1_filtered_with_normals_matlab_matched.las"
good = laspy.read(good_path)
bad  = laspy.read(bad_path)

def header_info(las):
    h = las.header
    return {
        "version": h.version,
        "point_format": h.point_format.id,
        "point_count": h.point_count,
        "scales": h.scales,
        "offsets": h.offsets,
        "mins": h.mins,
        "maxs": h.maxs,
        "software": h.generating_software,
        "system": h.system_identifier
    }

def stats(arr):
    return {
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr))
    }

print("\n========== HEADER COMPARISON ==========\n")
g = header_info(good)
b = header_info(bad)

for k in g:
    print(f"{k}")
    print(" good:", g[k])
    print(" bad :", b[k])
    print()

print("\n========== DIMENSION COMPARISON ==========\n")

g_dims = list(good.point_format.dimension_names)
b_dims = list(bad.point_format.dimension_names)

print("good dims:", g_dims)
print("bad dims :", b_dims)
print()

common_dims = sorted(set(g_dims) & set(b_dims))

print("\n========== FIELD STATISTICS ==========\n")

for dim in common_dims:
    g_vals = good[dim]
    b_vals = bad[dim]

    print(f"--- {dim} ---")
    print("good:", stats(g_vals))
    print("bad :", stats(b_vals))
    print()

print("\n========== NaN CHECK ==========\n")

for dim in common_dims:
    g_nan = np.isnan(good[dim]).sum() if good[dim].dtype.kind == 'f' else 0
    b_nan = np.isnan(bad[dim]).sum() if bad[dim].dtype.kind == 'f' else 0
    print(f"{dim}: good={g_nan}  bad={b_nan}")

print("\nDone.")