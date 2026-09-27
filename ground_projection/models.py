from dataclasses import dataclass, field
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Pixel:
    """Represents an image coordinate in pixels."""
    u: float
    v: float


@dataclass(frozen=True)
class DroneState:
    """
    Represents the drone's geodetic position and attitude.
    quaternion: [x, y, z, w] ordering (SciPy Rotation convention).
    altitude: Meters above ground level (AGL).
    """
    latitude: float
    longitude: float
    altitude: float
    quaternion: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics with optional distortion coefficients."""
    fx: float
    fy: float
    cx: float
    cy: float
    dist_coeffs: np.ndarray = field(default_factory=lambda: np.zeros(5, dtype=np.float64))


@dataclass(frozen=True)
class CameraMount:
    """
    Rigid mounting pose of the camera relative to the Drone Body Frame (FRD).
    position_body: [X_forward, Y_right, Z_down] in meters.
    rotation_body: Rotation from Body FRD to Camera Frame (OpenCV: X-right, Y-down, Z-forward).
    """
    position_body: np.ndarray
    rotation_body: Rotation


@dataclass(frozen=True)
class Ray:
    """A 3D ray defined by an origin point and a unit direction vector."""
    origin: np.ndarray
    direction: np.ndarray


@dataclass(frozen=True)
class GroundPoint:
    """Local ground intersection point in World NED coordinates (meters)."""
    north: float
    east: float
    down: float = 0.0


@dataclass(frozen=True)
class ProjectionResult:
    """Final projected geolocation and spatial metrics for an image pixel."""
    latitude: float
    longitude: float
    north: float
    east: float
    bearing: float
    euclidean: float
    manhattan: float