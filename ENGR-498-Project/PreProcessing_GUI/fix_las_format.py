import laspy
import numpy as np

inp = r"C:\Users\henry\Downloads\LAW2.las"
#C:\Users\henry\Downloads\movingtest1_filtered_with_normals_matlab.las"
out = r"C:\Users\henry\Downloads\LAW2_fixed.las"
#C:\Users\henry\Downloads\movingtest1_filtered_with_normals_matlab_fixed.las"

las = laspy.read(inp)

# --- force valid returns ---
las.return_number[:] = 1
las.number_of_returns[:] = 1

# --- remove RGB ---
for c in ("red","green","blue"):
    if c in las.point_format.dimension_names:
        getattr(las, c)[:] = 0

# --- match intensity distribution ---
i = las.intensity.astype(float)
i = (i - i.min()) / (i.max() - i.min() + 1e-9)
las.intensity = (i * 48000 + 2000).astype(np.uint16)

# --- shift coordinates to large-offset space (MATLAB-like) ---
x = np.asarray(las.x)
y = np.asarray(las.y)
z = np.asarray(las.z)

offset = np.array([300000, 6800000, 130])

las.header.offsets = offset
las.x = x + offset[0]
las.y = y + offset[1]
las.z = z + offset[2]

las.write(out)

print("FINAL FIXED:", out)