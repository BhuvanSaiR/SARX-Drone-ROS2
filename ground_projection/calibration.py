import numpy as np
import cv2
from .models import Pixel, CameraIntrinsics, Ray


def pixel_to_camera_ray(pixel: Pixel, intrinsics: CameraIntrinsics) -> Ray:
    """
    Undistorts a pixel coordinate and generates a normalized ray direction vector
    in the OpenCV camera frame.
    """
    camera_matrix = np.array([
        [intrinsics.fx, 0.0, intrinsics.cx],
        [0.0, intrinsics.fy, intrinsics.cy],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)

    raw_point = np.array([[[pixel.u, pixel.v]]], dtype=np.float64)

    # Undistort and normalize to image plane (z=1)
    undistorted = cv2.undistortPoints(
        raw_point,
        camera_matrix,
        intrinsics.dist_coeffs
    )

    x_norm = float(undistorted[0, 0, 0])
    y_norm = float(undistorted[0, 0, 1])

    direction = np.array([x_norm, y_norm, 1.0], dtype=np.float64)
    direction /= np.linalg.norm(direction)

    return Ray(
        origin=np.zeros(3, dtype=np.float64),
        direction=direction
    )