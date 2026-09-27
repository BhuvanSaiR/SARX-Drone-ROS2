import numpy as np

from models import GroundPoint
from models import ProjectionResult


class GPSUtils:
    """
    Local NED ↔ WGS84 conversion.

    Assumes a local tangent plane (ENU/NED approximation),
    which is appropriate for UAV operations over relatively
    small areas.
    """

    EARTH_RADIUS = 6378137.0

    # ---------------------------------------------------------

    @staticmethod
    def ned_to_gps(
        latitude,
        longitude,
        point: GroundPoint
    ):

        d_lat = point.north / GPSUtils.EARTH_RADIUS

        d_lon = (

            point.east

            /

            (

                GPSUtils.EARTH_RADIUS

                *

                np.cos(

                    np.radians(latitude)

                )

            )

        )

        lat = latitude + np.degrees(d_lat)

        lon = longitude + np.degrees(d_lon)

        return lat, lon

    # ---------------------------------------------------------

    @staticmethod
    def build_result(

        latitude,

        longitude,

        point,

        bearing,

        euclidean,

        manhattan

    ):

        lat, lon = GPSUtils.ned_to_gps(

            latitude,

            longitude,

            point

        )

        return ProjectionResult(

            latitude=lat,

            longitude=lon,

            north=point.north,

            east=point.east,

            bearing=bearing,

            euclidean=euclidean,

            manhattan=manhattan

        )