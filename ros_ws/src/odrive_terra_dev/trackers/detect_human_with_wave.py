from trackers import BaseHumanDetection
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Refactored DetectHumanWithWave Class
class DetectHumanWithWave(BaseHumanDetection):
    def __init__(self):
        super().__init__('detect_human_with_wave', '/image_in', '/image_out', '/detected_human')

        # Initialize HOG descriptor for human detection
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Load the OpenPose model
        self.proto_file = "pose_deploy_linevec.prototxt"
        self.weights_file = "pose_iter_440000.caffemodel"
        self.net = cv2.dnn.readNetFromCaffe(self.proto_file, self.weights_file)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self.frame_skip = 5  # Skip frames to reduce computational load
        self.frame_count = 0
        self.detected_keypoints = []

    def callback(self, data):
        self.frame_count += 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        if self.frame_count % self.frame_skip == 0:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            boxes, weights = self.hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)

            confidence_threshold = 0.9
            boxes = [box for i, box in enumerate(boxes) if weights[i] > confidence_threshold]
            weights = [weight for weight in weights if weight > confidence_threshold]
            boxes = [[x, y, x + w, y + h] for (x, y, w, h) in boxes]

            indices = cv2.dnn.NMSBoxes(boxes, weights, confidence_threshold, 0.4)
            if len(indices) > 0:
                indices = indices.flatten()

            self.detected_boxes = [boxes[i] for i in indices]

            point_out = Point()
            for (x1, y1, x2, y2) in self.detected_boxes:
                human_roi = cv_image[y1:y2, x1:x2]
                self.detected_keypoints = self.detect_wave(cv_image, human_roi, (x1, y1))
                if self.is_waving(self.detected_keypoints):
                    w = x2 - x1
                    h = y2 - y1
                    point_out.x = x1 + w / 2
                    point_out.y = y1 + h / 2
                    point_out.z = float(w * h) / (cv_image.shape[0] * cv_image.shape[1])

            if point_out.z > 0:
                normalized_point = self.normalize_coordinates(cv_image, point_out.x, point_out.y)
                normalized_point.z = point_out.z
                self.human_pub.publish(normalized_point)
                self.get_logger().info(f"Human detected waving at coordinates (x: {point_out.x}, y: {point_out.y}, z: {point_out.z})")

        self.draw_boxes(cv_image, self.detected_boxes)
        self.draw_keypoints(cv_image, self.detected_keypoints)

        img_to_pub = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        img_to_pub.header = data.header
        self.image_out_pub.publish(img_to_pub)

    def detect_wave(self, cv_image, roi, offset):
        in_height = 368
        in_width = int((in_height / roi.shape[0]) * roi.shape[1])
        inp_blob = cv2.dnn.blobFromImage(roi, 1.0 / 255, (in_width, in_height), (0, 0, 0), swapRB=False, crop=False)
        self.net.setInput(inp_blob)
        output = self.net.forward()

        H, W = output.shape[2], output.shape[3]
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
        if points[4] and points[3] and points[2]:
            if points[4][1] < points[2][1] and abs(points[4][0] - points[3][0]) < 30:
                return True
        return False

def main(args=None):
    rclpy.init(args=args)
    detect_human_with_wave = DetectHumanWithWave()
    rclpy.spin(detect_human_with_wave)
    detect_human_with_wave.destroy_node()
    rclpy.shutdown()