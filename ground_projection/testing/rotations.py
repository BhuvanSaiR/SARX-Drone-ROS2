import numpy as np


class RotationUtils:
    """
    Rotation utilities for a downward-facing camera on an
    ArduPilot drone.

    Drone Body Frame (FRD)
    ----------------------
    X = Forward
    Y = Right
    Z = Down

    OpenCV Camera Frame
    -------------------
    X = Right
    Y = Down
    Z = Optical Axis
    """

    def __init__(
        self,
        camera_roll_deg=0.0,
        camera_pitch_deg=30.0,
        camera_yaw_deg=0.0
    ):

        self.cam_roll = np.radians(camera_roll_deg)
        self.cam_pitch = np.radians(camera_pitch_deg)
        self.cam_yaw = np.radians(camera_yaw_deg)

    # ----------------------------------------------------
    # Basic Rotations
    # ----------------------------------------------------

    @staticmethod
    def Rx(a):

        c = np.cos(a)
        s = np.sin(a)

        return np.array([
            [1,0,0],
            [0,c,-s],
            [0,s,c]
        ])

    @staticmethod
    def Ry(a):

        c = np.cos(a)
        s = np.sin(a)

        return np.array([
            [ c,0,s],
            [ 0,1,0],
            [-s,0,c]
        ])

    @staticmethod
    def Rz(a):

        c = np.cos(a)
        s = np.sin(a)

        return np.array([
            [c,-s,0],
            [s, c,0],
            [0, 0,1]
        ])

    # ----------------------------------------------------
    # Drone attitude
    # ----------------------------------------------------

    def drone_rotation(self, roll, pitch, yaw):

        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

        Rx = self.Rx(roll)
        Ry = self.Ry(pitch)
        Rz = self.Rz(yaw)

        return Rz @ Ry @ Rx

    # ----------------------------------------------------
    # Camera Mount
    # ----------------------------------------------------

    def camera_mount_rotation(self):
        """
        Camera -> Drone body

        Camera is downward-facing.

        Image top points toward drone front.

        Camera then pitches 30°
        toward drone front.
        """

        # Camera frame -> Body frame
        #
        # Camera X(right)  -> Body Y(right)
        # Camera Y(down)   -> Body X(forward)
        # Camera Z(forward)-> Body Z(down)

        R_align = np.array([
            [0,1,0],
            [1,0,0],
            [0,0,1]
        ], dtype=float)

        Rx = self.Rx(self.cam_roll)
        Ry = self.Ry(-self.cam_pitch)
        Rz = self.Rz(self.cam_yaw)

        R_tilt = Rz @ Ry @ Rx

        return R_align @ R_tilt

    # ----------------------------------------------------
    # Camera -> World
    # ----------------------------------------------------

    def camera_to_world(
        self,
        roll,
        pitch,
        yaw
    ):

        R_body_world = self.drone_rotation(
            roll,
            pitch,
            yaw
        )

        R_cam_body = self.camera_mount_rotation()

        return R_body_world @ R_cam_body

    # ----------------------------------------------------
    # Rotate Ray
    # ----------------------------------------------------

    def rotate_ray(
        self,
        ray,
        roll,
        pitch,
        yaw
    ):

        R = self.camera_to_world(
            roll,
            pitch,
            yaw
        )

        world_ray = R @ ray

        world_ray /= np.linalg.norm(world_ray)

        return world_ray