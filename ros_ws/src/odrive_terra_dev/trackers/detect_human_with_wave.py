import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DetectHuman(Node):

    def __init__(self):
        super().__init__('detect_human')

        self.get_logger().info('Looking for a human...')
        self.image_sub = self.create_subscription(Image, "/image_in", self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.human_pub = self.create_publisher(Point, "/detected_human", 1)

        self.bridge = CvBridge()

        # Initialize HOG descriptor for human detection
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Load the OpenPose model
        self.proto_file = "pose_deploy_linevec.prototxt"
        self.weights_file = "pose_iter_440000.caffemodel"
        self.net = cv2.dnn.readNetFromCaffe(self.proto_file, self.weights_file)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        # Frame skipping to reduce computational load
        self.frame_skip = 5
        self.frame_count = 0

        # Store detected keypoints and boxes
        self.detected_keypoints = []
        self.detected_boxes = []

    def callback(self, data):
        self.frame_count += 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        if self.frame_count % self.frame_skip == 0:
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

                self.detected_boxes = [boxes[i] for i in indices]

                point_out = Point()

                # Check for waving gesture
                for (x1, y1, x2, y2) in self.detected_boxes:
                    human_roi = cv_image[y1:y2, x1:x2]

                    # Detect pose in the ROI and store keypoints
                    self.detected_keypoints = self.detect_wave(cv_image, human_roi, (x1, y1))

                    if self.is_waving(self.detected_keypoints):
                        w = x2 - x1
                        h = y2 - y1
                        point_out.x = x1 + w / 2
                        point_out.y = y1 + h / 2
                        point_out.z = float(w * h) / (cv_image.shape[0] * cv_image.shape[1])  # normalize area

                if point_out.z > 0:
                    normalized_point = self.normalize_coordinates(cv_image, point_out.x, point_out.y)
                    normalized_point.z = point_out.z
                    self.human_pub.publish(normalized_point)
                    self.get_logger().info(f"Human detected waving at normalized coordinates (x: {normalized_point.x}, y: {normalized_point.y}), z: {normalized_point.z}")

            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')

        # Draw keypoints and boxes on the original image
        self.draw_boxes(cv_image, self.detected_boxes)
        self.draw_keypoints(cv_image, self.detected_keypoints)

        # Always publish the image to avoid lag
        img_to_pub = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        img_to_pub.header = data.header
        self.image_out_pub.publish(img_to_pub)

    def detect_wave(self, cv_image, roi, offset):
        """Detect keypoints in the given region of interest (ROI) and return them."""
        in_height = 368
        in_width = int((in_height / roi.shape[0]) * roi.shape[1])
        inp_blob = cv2.dnn.blobFromImage(roi, 1.0 / 255, (in_width, in_height), (0, 0, 0), swapRB=False, crop=False)
        self.net.setInput(inp_blob)
        output = self.net.forward()

        H = output.shape[2]
        W = output.shape[3]

        points = []
        threshold = 0.1
        for i in range(15):
            heat_map = output[0, i, :, :]
            _, conf, _, point = cv2.minMaxLoc(heat_map)
            x = (roi.shape[1] * point[0]) / W
            y = (roi.shape[0] * point[1]) / H
            points.append((int(x) + offset[0], int(y) + offset[1]) if conf > threshold else None)

        return points

    def is_waving(self, points):
        """Check if the keypoints indicate a waving gesture."""
        if points[4] and points[3] and points[2]:  # Check if keypoints for right wrist, right elbow, and right shoulder are detected
            if points[4][1] < points[2][1] and abs(points[4][0] - points[3][0]) < 30:  # Right wrist is above right shoulder and close to right elbow
                return True
        return False

    def draw_keypoints(self, cv_image, points):
        """Draw keypoints on the image."""
        for point in points:
            if point:
                cv2.circle(cv_image, point, 5, (0, 255, 0), -1)

    def draw_boxes(self, cv_image, boxes):
        """Draw bounding boxes on the image."""
        for (x1, y1, x2, y2) in boxes:
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

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
