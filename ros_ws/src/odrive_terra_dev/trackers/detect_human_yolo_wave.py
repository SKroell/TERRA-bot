import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import time
from scipy.spatial import distance

class DetectHuman(Node):

    def __init__(self):
        super().__init__('detect_human')

        self.get_logger().info('Looking for a human...')
        self.image_sub = self.create_subscription(CompressedImage, "/image_in", self.callback, 10)
        self.image_out_pub = self.create_publisher(CompressedImage, "/image_out", 1)
        self.human_pub = self.create_publisher(Point, "/detected_human", 1)

        self.bridge = CvBridge()

        # Load YOLOv8 model with pose estimation
        self.model = YOLO("yolov8n-pose.pt")  # Use the yolov8n model (nano) for better speed
        self.conf_threshold = 0.5  # Confidence threshold
        self.nms_threshold = 0.4  # NMS threshold

        self.tracking_human = False
        self.tracked_human_id = None
        self.wait_time = 10  # seconds to wait before looking for a new human
        self.last_detection_time = time.time()

    def callback(self, data):
        current_time = time.time()
        if self.tracking_human and (current_time - self.last_detection_time < self.wait_time):
            return  # Skip processing if currently tracking a human and within wait time

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
            keypoints = results[0].keypoints.cpu().numpy()  # Keypoints for pose estimation

            detected_humans = []

            for box, confidence, class_id, kps in zip(boxes, confidences, class_ids, keypoints):
                if confidence > self.conf_threshold and int(class_id) == 0:  # class_id == 0 for 'person'
                    x1, y1, x2, y2 = box.astype(int)
                    detected_humans.append({
                        'box': [x1, y1, x2 - x1, y2 - y1],
                        'confidence': float(confidence),
                        'keypoints': kps,
                        'id': None  # Placeholder for unique ID
                    })
                    label = f'Person: {confidence:.2f}'
                    cv_image = self.draw_keypoints(cv_image, kps)
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Create CompressedImage for visualization
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
            self.image_out_pub.publish(msg)

            if self.tracking_human:
                # Track the same human based on the closest keypoints match
                tracked_human = self.track_human(detected_humans)
                if tracked_human:
                    self.update_tracking(tracked_human)
            else:
                # Check for waving gesture in detected humans and start tracking
                for human in detected_humans:
                    if self.is_waving(human['keypoints']):
                        self.start_tracking(human, current_time)
                        break

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def track_human(self, detected_humans):
        """Track the same human based on keypoints."""
        if self.tracked_human_id is None:
            return None

        min_dist = float('inf')
        tracked_human = None
        for human in detected_humans:
            dist = distance.euclidean(human['keypoints'].flatten(), self.tracked_keypoints.flatten())
            if dist < min_dist:
                min_dist = dist
                tracked_human = human

        if tracked_human and min_dist < self.nms_threshold:
            return tracked_human
        return None

    def start_tracking(self, human, current_time):
        """Start tracking a new human."""
        self.tracking_human = True
        self.tracked_human_id = id(human)
        self.tracked_keypoints = human['keypoints']
        self.update_tracking(human)
        self.last_detection_time = current_time

    def update_tracking(self, human):
        """Update tracking information."""
        box = human['box']
        point_out = Point()
        point_out.x = box[0] + box[2] / 2
        point_out.y = box[1] + box[3] / 2
        point_out.z = float(box[2] * box[3])
        normalized_point = self.normalize_coordinates(self.cv_image, point_out.x, point_out.y)
        self.human_pub.publish(normalized_point)
        self.get_logger().info(f"Tracking human ID {self.tracked_human_id} at coordinates (x: {point_out.x}, y: {point_out.y}, z: {point_out.z})")

    def is_waving(self, keypoints):
        """Detect if the person is waving using keypoints from pose estimation."""
        left_wrist = keypoints[9]  # Assuming keypoints[9] is left wrist
        right_wrist = keypoints[10]  # Assuming keypoints[10] is right wrist
        nose = keypoints[0]  # Assuming keypoints[0] is nose

        # Simple heuristic: wrist above nose level indicates a waving gesture
        if left_wrist[1] < nose[1] or right_wrist[1] < nose[1]:
            return True
        return False

    def draw_keypoints(self, image, keypoints):
        """Draw keypoints on the image."""
        for kp in keypoints:
            x, y = int(kp[0]), int(kp[1])
            cv2.circle(image, (x, y), 3, (0, 255, 0), -1)
        return image

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
