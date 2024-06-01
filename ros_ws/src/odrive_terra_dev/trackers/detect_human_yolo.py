import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO

class DetectHuman(Node):

    def __init__(self):
        super().__init__('detect_human')

        self.get_logger().info('Looking for a human...')
        self.image_sub = self.create_subscription(CompressedImage, "/image_in", self.callback, 10)
        self.image_out_pub = self.create_publisher(CompressedImage, "/image_out", 1)
        self.human_pub = self.create_publisher(Point, "/detected_human", 1)

        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt")  # Use the yolov8n model (nano) for better speed
        self.conf_threshold = 0.5  # Confidence threshold
        self.nms_threshold = 0.4  # NMS threshold

    def callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        try:
            results = self.model(cv_image)
            boxes = results[0].boxes.xyxy.cpu().numpy()  # Bounding boxes
            confidences = results[0].boxes.conf.cpu().numpy()  # Confidences
            class_ids = results[0].boxes.cls.cpu().numpy()  # Class IDs
            class_names = results[0].names  # Class names

            human_boxes = []
            human_confidences = []

            for box, confidence, class_id in zip(boxes, confidences, class_ids):
                if confidence > self.conf_threshold:
                    x1, y1, x2, y2 = box.astype(int)
                    if int(class_id) == 0:  # class_id == 0 for 'person'
                        human_boxes.append([x1, y1, x2 - x1, y2 - y1])
                        human_confidences.append(float(confidence))
                    label = f'{class_names[int(class_id)]}: {confidence:.2f}'
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Create CompressedImage
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
            self.image_out_pub.publish(msg)

            point_out = Point()

            # Keep the biggest detected human
            if human_boxes:
                human_indices = cv2.dnn.NMSBoxes(human_boxes, human_confidences, self.conf_threshold, self.nms_threshold)
                if human_indices is not None and len(human_indices) > 0:
                    for i in human_indices:
                        if isinstance(i, (list, tuple)):
                            i = i[0]
                        box = human_boxes[i]
                        x, y, w, h = box[0], box[1], box[2], box[3]
                        if w * h > point_out.z:
                            point_out.x = x + w / 2
                            point_out.y = y + h / 2
                            point_out.z = float(w * h)

            if point_out.z > 0:
                normalized_point = self.normalize_coordinates(cv_image, point_out.x, point_out.y)
                self.human_pub.publish(normalized_point)
                self.get_logger().info(f"Human detected at coordinates (x: {point_out.x}, y: {point_out.y}, z: {point_out.z})")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def normalize_coordinates(self, image, x, y, z):
        """Normalize the coordinates based on image dimensions."""
        height, width, _ = image.shape
        normalized_x = (x - (width / 2)) / (width / 2)
        normalized_y = (y - (height / 2)) / (height / 2)
        normalized_z = z / (width * height)
        return Point(x=normalized_x, y=normalized_y, z=1.0)

def main(args=None):
    rclpy.init(args=args)
    detect_human = DetectHuman()
    rclpy.spin(detect_human)
    detect_human.destroy_node()
    rclpy.shutdown()
