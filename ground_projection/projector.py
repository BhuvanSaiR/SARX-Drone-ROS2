import numpy as np
from .models import (
    Pixel,
    DroneState,
    CameraIntrinsics,
    CameraMount,
    ProjectionResult,
)
from .calibration import pixel_to_camera_ray
from .transforms import (
    get_body_to_camera_transform,
    get_world_to_body_transform,
    transform_ray_to_world,
)
from .projection import intersect_ray_ground
from .gps import ned_to_gps, compute_bearing, compute_distances


class GroundProjector:
    """
    Main projection engine that maps image pixels to geodetic ground coordinates.
    """
    def __init__(self, intrinsics: CameraIntrinsics, mount: CameraMount):
        self.intrinsics = intrinsics
        self.mount = mount
        self.T_body_camera = get_body_to_camera_transform(mount)

    def project(self, pixel: Pixel, drone_state: DroneState) -> ProjectionResult:
        """
        Executes the end-to-end projection pipeline for a single pixel.
        """
        # 1. Image -> Camera Ray (OpenCV frame)
        ray_cam = pixel_to_camera_ray(pixel, self.intrinsics)

        # 2. Compose Homogeneous Transforms: T_world_camera = T_world_body @ T_body_camera
        T_world_body = get_world_to_body_transform(drone_state)
        T_world_camera = T_world_body @ self.T_body_camera

        # 3. Transform Ray -> World NED Frame
        ray_world = transform_ray_to_world(ray_cam, T_world_camera)

        # 4. Ray-Ground Intersection (Z = 0)
        ground_point = intersect_ray_ground(ray_world, ground_z=0.0)

        # 5. Geodetic & Spatial Metrics
        lat, lon = ned_to_gps(
            drone_state.latitude,
            drone_state.longitude,
            ground_point
        )
        bearing = compute_bearing(ground_point.north, ground_point.east)
        euclidean, manhattan = compute_distances(ground_point.north, ground_point.east)

        return ProjectionResult(
            latitude=lat,
            longitude=lon,
            north=ground_point.north,
            east=ground_point.east,
            bearing=bearing,
            euclidean=euclidean,
            manhattan=manhattan
        )