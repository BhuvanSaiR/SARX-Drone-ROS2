import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

# ---- Import your YOLO utils ----
from models.yolo import Model
from utils.general import non_max_suppression, scale_boxes, check_yaml
from utils.torch_utils import select_device


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        self.bridge = CvBridge()

        # Subscribe to FRONT camera
        self.sub = self.create_subscription(
            Image,
            '/camera/front',
            self.front_callback,
            10
        )

        # Publish detection result
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/detection/front',
            10
        )

        # ---- Load YOUR model ----
        self.device = select_device("cpu")

        cfg = check_yaml("/home/drone/Desktop/yolov5/models/yolov5n.yaml")
        self.model = Model(cfg, ch=3, nc=1)

        state_dict = torch.load(
            "/home/drone/Desktop/final/weights_only.pt",
            map_location="cpu"
        )

        self.model.load_state_dict(state_dict, strict=True)
        self.model.to(self.device).eval()

        self.get_logger().info("YOLO model loaded")

        self.IMG_SIZE = 320
        self.CONF_THRES = 0.25
        self.IOU_THRES = 0.45
        self.sub_bottom = self.create_subscription(
            Image,
            '/camera/bottom',
            self.bottom_callback,
            10
        )

        self.pub_bottom = self.create_publisher(
            Float32MultiArray,
            '/detection/bottom',
            10
        )

    def front_callback(self, msg):
        self.process_frame(msg, self.pub)

    def bottom_callback(self, msg):
        self.process_frame(msg, self.pub_bottom)
    
    # def process_frame(self, msg, publisher):
        # # Convert ROS Image to OpenCV
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # # Preprocess for YOLO
        # img = cv2.resize(cv_image, (self.IMG_SIZE, self.IMG_SIZE))
        # img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB and HWC to CHW
        # img = np.ascontiguousarray(img) / 255.0  # Normalize

        # with torch.no_grad():
        #     input_tensor = torch.from_numpy(img).float().unsqueeze(0).to(self.device)
        #     pred = self.model(input_tensor)[0]
        #     pred = non_max_suppression(pred, self.CONF_THRES, self.IOU_THRES)

        # detections = []
        # if pred[0] is not None:
        #     for *box, conf, cls in pred[0].cpu().numpy():
        #         x_center = (box[0] + box[2]) / 2
        #         y_center = (box[1] + box[3]) / 2
        #         detections.append((x_center, y_center))

        # # Publish detections as Float32MultiArray
        # msg_out = Float32MultiArray()
        # msg_out.data = [coord for det in detections for coord in det]
        # publisher.publish(msg_out)
    def process_frame(self, msg, publisher):

        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        orig_h, orig_w = frame.shape[:2]

        # ---- PREPROCESS ----
        img = cv2.resize(frame, (self.IMG_SIZE, self.IMG_SIZE))
        img = img.transpose(2, 0, 1)

        img = torch.from_numpy(img).to(self.device)
        img = img.float() / 255.0
        img = img.unsqueeze(0)

        # ---- INFERENCE ----
        with torch.no_grad():
            pred = self.model(img)

        pred = non_max_suppression(
            pred,
            conf_thres=self.CONF_THRES,
            iou_thres=self.IOU_THRES
        )

        # ---- EXTRACT INFO ----
        area_ratio = 0.0
        cx = 0.0
        cy = 0.0
        found = 0.0

        det = pred[0] if len(pred) > 0 else []

        if len(det):
            det[:, :4] = scale_boxes(
                img.shape[2:], det[:, :4], frame.shape
            ).round()

            max_area = 0

            for *xyxy, conf, cls in det:

                if int(cls) != 0:
                    continue

                x1, y1, x2, y2 = map(float, xyxy)

                w = max(0, x2 - x1)
                h = max(0, y2 - y1)
                area = w * h

                if area > max_area:
                    max_area = area

                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    cx = (center_x - orig_w/2) / (orig_w/2)
                    cy = (center_y - orig_h/2) / (orig_h/2)

                    found = 1.0

            area_ratio = max_area / (orig_w * orig_h)

        # ---- PUBLISH ----
        msg_out = Float32MultiArray()
        msg_out.data = [found, area_ratio, cx, cy]

        publisher.publish(msg_out)

def main():
    rclpy.init()
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()