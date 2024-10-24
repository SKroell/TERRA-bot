import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# HOG-Based Human Detection Class
class DetectHumanHOG(BaseHumanDetection):
    def __init__(self):
        super().__init__('detect_human_hog', '/image_in', '/image_out', '/detected_human')
        
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

        # Convert to grayscale for better detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect humans in the image
        boxes, weights = self.hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)

        # Filter detections based on confidence threshold
        confidence_threshold = 0.9
        boxes = [box for i, box in enumerate(boxes) if weights[i] > confidence_threshold]
        weights = [weight for weight in weights if weight > confidence_threshold]

        # Apply Non-Maximum Suppression (NMS)
        boxes = [[x, y, x + w, y + h] for (x, y, w, h) in boxes]
        indices = cv2.dnn.NMSBoxes(boxes, weights, confidence_threshold, 0.4)
        if len(indices) > 0:
            indices = indices.flatten()

        boxes = [boxes[i] for i in indices]

        for (x1, y1, x2, y2) in boxes:
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Create CompressedIamge and publish
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

def main(args=None):
    rclpy.init(args=args)
    detect_human_hog = DetectHumanHOG()
    rclpy.spin(detect_human_hog)
    detect_human_hog.destroy_node()
    rclpy.shutdown()