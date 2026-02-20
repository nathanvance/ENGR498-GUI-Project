import cv2
import glob
import numpy as np

# Checkerboard settings
CHECKERBOARD = (9, 6)      # internal corners
SQUARE_SIZE = 0.025       # meters (25 mm)

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

images = glob.glob("calib_images/*.jpg")

for img_path in images:
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

np.savez("camera_intrinsics.npz", K=K, dist=dist)
print("Camera matrix:\n", K)
