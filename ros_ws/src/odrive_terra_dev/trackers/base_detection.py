import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Base Detection Class
class BaseHumanDetection(Node):
    def __init__(self, node_name, sub_topic, pub_image_topic, pub_human_topic):
        super().__init__(node_name)

        self.get_logger().info(f'{node_name} initialized and looking for a human...')
        self.image_sub = self.create_subscription(CompressedImage, sub_topic, self.callback, 10)
        self.image_out_pub = self.create_publisher(CompressedImage, pub_image_topic, 1)
        self.human_pub = self.create_publisher(Point, pub_human_topic, 1)

        self.bridge = CvBridge()
        self.conf_threshold = 0.5
        self.nms_threshold = 0.4

    def callback(self, data):
        """Placeholder method to be overridden by child classes."""
        raise NotImplementedError("Subclasses must implement the callback method for specific detection logic")

    def normalize_coordinates(self, image, x, y):
        """Normalize the coordinates based on image dimensions."""
        height, width, _ = image.shape
        normalized_x = (x - (width / 2)) / (width / 2)
        normalized_y = (y - (height / 2)) / (height / 2)
        return Point(x=normalized_x, y=normalized_y, z=1.0)

    def callback(self, data):
        raise NotImplementedError("Subclasses should implement this method.")

    def draw_keypoints(self, keypoints, box):
        """Draw keypoints on the image."""
        for index, kp in enumerate(keypoints):
            #print(kp)
            x, y = int(kp[0]), int(kp[1])
            cv2.circle(self.cv_image, (x, y), radius=3, color=(0, 255, 0), thickness=-1)
            # write index of keypoint on the image
            cv2.putText(self.cv_image, str(index), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    def normalize_coordinates(self, image, x, y):
        """Normalize the coordinates based on image dimensions."""
        height, width, _ = image.shape
        normalized_x = (x - (width / 2)) / (width / 2)
        normalized_y = (y - (height / 2)) / (height / 2)
        return Point(x=normalized_x, y=normalized_y, z=1.0)

# Uncomment to execute in an appropriate ROS environment
# main_human_wave()
# main_aruco()
# main_hog()
# main_yolo()

