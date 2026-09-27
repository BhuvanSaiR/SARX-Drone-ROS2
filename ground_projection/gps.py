import math
from .models import GroundPoint


# WGS84 Ellipsoid Constants
WGS84_A = 6378137.0             # Semi-major axis (meters)
WGS84_E2 = 0.006694379990141316  # First eccentricity squared


def get_radii_of_curvature(lat_rad: float) -> tuple[float, float]:
    """Computes Meridian (M) and Prime Vertical (N) radii of curvature at a latitude."""
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * (sin_lat ** 2))
    
    M = (WGS84_A * (1.0 - WGS84_E2)) / (denom ** 3)
    N = WGS84_A / denom
    return M, N


def ned_to_gps(
    drone_lat: float,
    drone_lon: float,
    ground_point: GroundPoint
) -> tuple[float, float]:
    """
    Converts local North/East offsets (meters) in NED to target Latitude/Longitude (degrees)
    using WGS84 radii of curvature.
    """
    lat_rad = math.radians(drone_lat)
    M, N = get_radii_of_curvature(lat_rad)

    delta_lat_rad = ground_point.north / M
    delta_lon_rad = ground_point.east / (N * math.cos(lat_rad))

    target_lat = drone_lat + math.degrees(delta_lat_rad)
    target_lon = drone_lon + math.degrees(delta_lon_rad)

    return target_lat, target_lon


def compute_bearing(north: float, east: float) -> float:
    """Computes clockwise compass bearing from North (0° to 360°)."""
    bearing_deg = math.degrees(math.atan2(east, north))
    return (bearing_deg + 360.0) % 360.0


def compute_distances(north: float, east: float) -> tuple[float, float]:
    """Returns (euclidean_distance, manhattan_distance) in meters."""
    euclidean = math.hypot(north, east)
    manhattan = abs(north) + abs(east)
    return euclidean, manhattan