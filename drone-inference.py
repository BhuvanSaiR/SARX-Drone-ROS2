"""Object detection for images (Ultralytics YOLO) + Ground Geolocation.
Filters for humans only, calculates ground coordinates based on drone altitude,
and outputs an annotated image.

Examples:
    python drone-inference.py --source input.jpg --altitude 50.0
"""

import cv2
import argparse
import numpy as np
from scipy.spatial.transform import Rotation
from ultralytics import YOLO
from ultralytics.utils.plotting import colors

# --- Ground Projection Imports ---
from ground_projection.models import (
    Pixel,
    DroneState,
    CameraIntrinsics,
    CameraMount,
)
from ground_projection.projector import GroundProjector
from ground_projection.projection import ProjectionError


class ImageVision:
    """Detect humans in an image with a YOLO model and project to ground."""

    def __init__(self, args):
        """Load the model and initialize the projector."""
        self.args = args
        self.model = YOLO(args.weights)
        self.names = self.model.names
        self.scale = None
        
        # Initialize the projection pipeline
        self.projector = self._setup_projector()

    def _setup_projector(self) -> GroundProjector:
        """Configures the camera intrinsics and mount."""
        # 1. Camera Intrinsics (Matches demo.py)
        intrinsics = CameraIntrinsics(
            fx=1200.0,
            fy=1200.0,
            cx=960.0,
            cy=540.0,
            dist_coeffs=np.zeros(5)
        )

        # 2. Camera Mount Calibration
        GPS_TO_FC = np.array([0.07, 0.13, 0.05])
        FC_TO_CAMERA = np.array([0.20, 0.00, 0.10])
        GPS_TO_CAMERA = GPS_TO_FC + FC_TO_CAMERA

        # Downward-looking camera fix: Roll=0, Pitch=0, Yaw=90
        mount_rotation = Rotation.from_euler("xyz", [0.0, 0.0, 90.0], degrees=True)
        mount = CameraMount(position_body=GPS_TO_CAMERA, rotation_body=mount_rotation)

        return GroundProjector(intrinsics=intrinsics, mount=mount)

    def run(self):
        """Process the image, annotate humans, and show or save it."""
        a = self.args

        # Predict only class 0 (Person/Human)
        results = self.model.predict(
            source=a.source, 
            conf=a.conf, 
            iou=a.iou, 
            imgsz=a.imgsz,
            device=a.device, 
            classes=[0], # 0 is the COCO class index for 'person'
            verbose=False
        )

        # Since we are processing a single image, we take the first result
        res = results[0]
        frame = res.orig_img
        h, w = frame.shape[:2]

        # Dynamically set bounding box and text scales based on image resolution
        short = min(w, h)
        self.scale = (max(1, round(short / 360)),
                      max(0.4, short / 1200),
                      max(1, round(short / 600)))

        # Annotate the image with projections
        self._annotate(frame, res.boxes)

        # Save the image
        if a.save:
            cv2.imwrite(a.out, frame)
            print(f"\n[i] Saved annotated image to {a.out}")

        # Show the image
        if not a.no_show:
            cv2.imshow("Human Detection & Geolocation", frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def _annotate(self, frame, boxes):
        """Draw bounding boxes, calculate target distances & GPS, and label humans."""
        line, font, ft = self.scale

        # Setup the Drone State using the altitude passed from the command line
        drone_state = DroneState(
            latitude=17.385044,      # Arbitrary Location: Hyderabad, Telangana
            longitude=78.486671,     
            altitude=self.args.altitude,
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]) # Assume hovering level for now
        )

        print(f"--- Running Projection (Altitude: {self.args.altitude}m) ---")
        print(f"Drone Origin GPS: {drone_state.latitude:.6f}, {drone_state.longitude:.6f}\n")

        if boxes is not None and len(boxes):
            xyxy = boxes.xyxy.cpu().numpy()
            cls = boxes.cls.int().cpu().tolist()
            conf = boxes.conf.cpu().tolist()

            for i, c in enumerate(cls):
                x1, y1, x2, y2 = xyxy[i].astype(int)
                
                # --- CALCULATE U,V ---
                # Get the pixel at the bottom center of the bounding box (the feet)
                u = (x1 + x2) / 2.0
                v = float(y2)

                try:
                    # Project to ground
                    pixel = Pixel(u=u, v=v)
                    result = self.projector.project(pixel, drone_state)
                    
                    dist = result.euclidean
                    lat = result.latitude
                    lon = result.longitude
                    
                    # Update label with Distance and GPS
                    label = f"{self.names[c]} {conf[i]:.2f} | {dist:.1f}m | {lat:.6f}, {lon:.6f}"
                    
                    print(f"[+] Target {i+1} found at pixel ({int(u)}, {int(v)})")
                    print(f"    -> Real-World GPS: {lat:.7f}, {lon:.7f}")
                    print(f"    -> Distance from Drone: {dist:.1f}m (North: {result.north:.1f}m, East: {result.east:.1f}m)")
                except ProjectionError as e:
                    label = f"{self.names[c]} {conf[i]:.2f} | Proj Err"
                    print(f"[!] Target {i+1} at pixel ({int(u)}, {int(v)}) failed projection: {e}")

                color = colors(c, True)

                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, line, cv2.LINE_AA)
                
                # Draw a dot exactly at the (u,v) target (the feet)
                cv2.circle(frame, (int(u), int(v)), radius=line * 2, color=(0, 0, 255), thickness=-1)
                
                # Draw label background and text
                (tw, th), base = cv2.getTextSize(label, 0, font, ft)
                pad = max(3, round(th * 0.35))
                cv2.rectangle(frame, (x1, max(0, y1 - th - base - pad)),
                              (x1 + tw + 2 * pad, y1), color, -1, cv2.LINE_AA)
                cv2.putText(frame, label, (x1 + pad, y1 - base), 0,
                            font, (255, 255, 255), ft, cv2.LINE_AA)


def parse_args():
    """Parse command-line arguments."""
    p = argparse.ArgumentParser(description="Ultralytics YOLO Image Human Inference & Projection")
    p.add_argument("--weights", default="yolov8n.pt", help="path to trained model")
    p.add_argument("--source", required=True, help="path to the input image")
    p.add_argument("--altitude", type=float, required=True, help="drone altitude in meters")
    p.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    p.add_argument("--iou", type=float, default=0.5, help="NMS IoU threshold")
    p.add_argument("--imgsz", type=int, default=640, help="inference size")
    p.add_argument("--device", default=None, help="cuda device, e.g. 0 or cpu")
    
    # Save behaviour arguments
    p.add_argument("--save", action="store_true", default=True, help="write annotated image to disk (Default: True)")
    p.add_argument("--no-show", action="store_true", help="don't open a display window")
    p.add_argument("--out", default="output.jpg", help="output path for the saved image")
    
    return p.parse_args()


if __name__ == "__main__":
    ImageVision(parse_args()).run()