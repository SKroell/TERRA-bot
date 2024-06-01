import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DetectHuman(Node):

    def __init__(self):
        super().__init__('detect_human')

        self.get_logger().info('Looking for a human...')
        self.image_sub = self.create_subscription(CompressedImage, "/image_in", self.callback, 10)
        self.image_out_pub = self.create_publisher(CompressedImage, "/image_out", 1)
        self.human_pub = self.create_publisher(Point, "/detected_human", 1)

        self.bridge = CvBridge()

        # Initialize HOG descriptor for human detection
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        try:
            # Convert to grayscale for better detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect humans in the image
            boxes, weights = self.hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)

            # Apply a higher confidence threshold to filter out weak detections
            confidence_threshold = 0.9
            boxes = [box for i, box in enumerate(boxes) if weights[i] > confidence_threshold]
            weights = [weight for weight in weights if weight > confidence_threshold]

            # Convert boxes to the format expected by NMS
            boxes = [[x, y, x + w, y + h] for (x, y, w, h) in boxes]

            # Apply Non-Maximum Suppression (NMS)
            indices = cv2.dnn.NMSBoxes(boxes, weights, confidence_threshold, 0.4)
            if len(indices) > 0:
                indices = indices.flatten()

            boxes = [boxes[i] for i in indices]

            for (x1, y1, x2, y2) in boxes:
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

            #### Create CompressedIamge ####
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
            self.image_out_pub.publish(msg)

            point_out = Point()

            # Keep the biggest detected human
            for (x1, y1, x2, y2) in boxes:
                w = x2 - x1
                h = y2 - y1
                if w * h > point_out.z:
                    point_out.x = x1 + w / 2
                    point_out.y = y1 + h / 2
                    point_out.z = float(w * h) / (cv_image.shape[0] * cv_image.shape[1])  # normalize area

            if point_out.z > 0:
                normalized_point = self.normalize_coordinates(cv_image, point_out.x, point_out.y)
                normalized_point.z = point_out.z
                self.human_pub.publish(normalized_point)
                self.get_logger().info(f"Human detected at coordinates (x: {point_out.x}, y: {point_out.y}, z: {point_out.z})")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def normalize_coordinates(self, image, x, y):
        """Normalize the coordinates based on image dimensions."""
        height, width, _ = image.shape
        normalized_x = (x - (width / 2)) / (width / 2)
        normalized_y = (y - (height / 2)) / (height / 2)
        return Point(x=normalized_x, y=normalized_y, z=1.0)

def main(args=None):
    rclpy.init(args=args)
    detect_human = DetectHuman()
    rclpy.spin(detect_human)
    detect_human.destroy_node()
    rclpy.shutdown()
