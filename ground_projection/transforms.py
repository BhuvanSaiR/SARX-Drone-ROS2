import numpy as np
from scipy.spatial.transform import Rotation
from .models import CameraMount, DroneState, Ray


def make_transform(rotation: Rotation, translation: np.ndarray) -> np.ndarray:
    """Constructs a 4x4 homogeneous transformation matrix."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rotation.as_matrix()
    T[:3, 3] = translation
    return T


def apply_transform_point(T: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Applies a 4x4 transform to a 3D point (homogeneous coordinate w=1)."""
    h_point = np.append(point, 1.0)
    return (T @ h_point)[:3]


def apply_transform_vector(T: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Applies a 4x4 transform to a 3D direction vector (w=0, rotation only)."""
    return T[:3, :3] @ vector


def get_body_to_camera_transform(mount: CameraMount) -> np.ndarray:
    """Returns T_body_camera (Camera frame to Body FRD frame)."""
    return make_transform(mount.rotation_body, mount.position_body)


def get_world_to_body_transform(drone_state: DroneState) -> np.ndarray:
    """
    Returns T_world_body (Body FRD frame to World NED frame).
    In NED, Z is positive downwards. Altitude above ground (AGL) corresponds
    to a negative Z coordinate in World NED (-altitude).
    """
    rotation = Rotation.from_quat(drone_state.quaternion)
    translation = np.array([0.0, 0.0, -drone_state.altitude], dtype=np.float64)
    return make_transform(rotation, translation)


def transform_ray_to_world(ray_camera: Ray, T_world_camera: np.ndarray) -> Ray:
    """Transforms a Ray from the camera frame into the World NED frame."""
    origin_world = apply_transform_point(T_world_camera, ray_camera.origin)
    direction_world = apply_transform_vector(T_world_camera, ray_camera.direction)
    direction_world /= np.linalg.norm(direction_world)
    return Ray(origin=origin_world, direction=direction_world)