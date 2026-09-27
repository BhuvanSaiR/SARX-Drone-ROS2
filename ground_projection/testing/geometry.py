import numpy as np

from models import Ray
from models import GroundPoint


class Geometry:

    """
    Pure geometry.

    No GPS.

    No OpenCV.

    No Drone.
    """

    GROUND_NORMAL = np.array([
        0,
        0,
        1
    ])

    GROUND_POINT = np.zeros(3)

    # -----------------------------------------------------

    @staticmethod
    def ray_plane_intersection(

        ray: Ray,

        plane_normal,

        plane_point

    ):

        d = np.dot(

            plane_normal,

            ray.direction

        )

        if abs(d) < 1e-8:

            return None

        t = np.dot(

            plane_normal,

            plane_point - ray.origin

        ) / d

        if t < 0:

            return None

        return ray.origin + t * ray.direction

    # -----------------------------------------------------

    @staticmethod
    def intersect_ground(

        ray: Ray

    ):

        p = Geometry.ray_plane_intersection(

            ray,

            Geometry.GROUND_NORMAL,

            Geometry.GROUND_POINT

        )

        if p is None:

            return None

        return GroundPoint(

            north=p[0],

            east=p[1],

            down=0

        )

    # -----------------------------------------------------

    @staticmethod
    def bearing(

        point: GroundPoint

    ):

        return (

            np.degrees(

                np.arctan2(

                    point.east,

                    point.north

                )

            )

            + 360

        ) % 360

    # -----------------------------------------------------

    @staticmethod
    def euclidean(

        point: GroundPoint

    ):

        return np.hypot(

            point.north,

            point.east

        )

    # -----------------------------------------------------

    @staticmethod
    def manhattan(

        point: GroundPoint

    ):

        return (

            abs(point.north)

            +

            abs(point.east)

        )