import numpy as np
from scipy.spatial.transform import Rotation
from ground_projection.models import (
    Pixel,
    DroneState,
    CameraIntrinsics,
    CameraMount,
)
from ground_projection.projector import GroundProjector


# --- 1. Physical Mount Calibration (One-Source-of-Truth) ---
GPS_TO_FC = np.array([0.07, 0.13, 0.05])
FC_TO_CAMERA = np.array([0.20, 0.00, 0.10])
GPS_TO_CAMERA = GPS_TO_FC + FC_TO_CAMERA

# Downward-looking camera: 
# Map OpenCV Z (Forward/Lens) -> FRD Z (Down)
# To orient the top of the image towards the front of the drone, we rotate Yaw by 90 deg.
# Roll=0, Pitch=0, Yaw=90
mount_rotation = Rotation.from_euler("xyz", [0.0, 0.0, 90.0], degrees=True)

mount = CameraMount(
    position_body=GPS_TO_CAMERA,
    rotation_body=mount_rotation
)

# --- 2. Camera Intrinsics (e.g., 1080p camera) ---
intrinsics = CameraIntrinsics(
    fx=1200.0,
    fy=1200.0,
    cx=960.0,
    cy=540.0,
    dist_coeffs=np.zeros(5)
)

# --- 3. Initialize Projector ---
projector = GroundProjector(intrinsics=intrinsics, mount=mount)

# --- 4. Define Drone State & Target Pixel ---
# Drone hovering at 100m altitude AGL, level flight (quaternion = [0, 0, 0, 1])
drone_state = DroneState(
    latitude=17.385044,     # Hyderabad, Telangana
    longitude=78.486671,
    altitude=100.0,
    quaternion=np.array([0.0, 0.0, 0.0, 1.0])
)

# Target pixel at bottom-right quadrant of image
pixel = Pixel(u=1100.0, v=700.0)

# --- 5. Run Projection ---
result = projector.project(pixel, drone_state)

print("=== Projection Result ===")
print(f"Target GPS    : {result.latitude:.8f}° N, {result.longitude:.8f}° E")
print(f"Local NED     : North = {result.north:.2f} m | East = {result.east:.2f} m")
print(f"Bearing       : {result.bearing:.1f}°")
print(f"Distance (Euc): {result.euclidean:.2f} m")
print(f"Distance (Man): {result.manhattan:.2f} m")