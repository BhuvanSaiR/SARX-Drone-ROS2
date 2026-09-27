import numpy as np
from .models import Ray, GroundPoint


class ProjectionError(Exception):
    """Raised when a ray does not intersect the target ground plane."""
    pass


def intersect_ray_ground(ray_world: Ray, ground_z: float = 0.0) -> GroundPoint:
    """
    Computes the intersection of a ray in NED frame with the ground plane (Z = 0).
    In NED, Z points down, so the ground plane is at Z = 0 and the drone is at Z < 0.
    """
    dz = ray_world.direction[2]
    
    # Check if ray is pointing parallel to or away from the ground
    if dz <= 1e-6:
        raise ProjectionError("Ray is parallel to or pointing away from the ground plane.")

    # Solve for scale parameter t where origin_z + t * dz = ground_z
    t = (ground_z - ray_world.origin[2]) / dz

    if t < 0:
        raise ProjectionError("Ray intersection occurs behind the camera origin.")

    intersection = ray_world.origin + t * ray_world.direction

    return GroundPoint(
        north=float(intersection[0]),
        east=float(intersection[1]),
        down=float(intersection[2])
    )