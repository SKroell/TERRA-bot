import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco

c# Refactored DetectAruco Class
class DetectAruco(BaseHumanDetection):
    def __init__(self):
        super().__init__('detect_aruco', '/image_in', '/image_out', '/detected_aruco')

        # Load the predefined ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        try:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

                point_out = Point()
                for corner in corners:
                    corner = corner[0]
                    center_x = int((corner[0][0] + corner[2][0]) / 2)
                    center_y = int((corner[0][1] + corner[2][1]) / 2)
                    point_out.x = center_x
                    point_out.y = center_y

                    marker_width = corner[2][0] - corner[0][0]
                    marker_height = corner[2][1] - corner[0][1]
                    marker_area = marker_width * marker_height

                    image_height, image_width = cv_image.shape[:2]
                    image_area = image_width * image_height

                    point_out.z = marker_area / image_area
                    cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    normalized_point = self.normalize_coordinates(cv_image, point_out.x, point_out.y)
                    normalized_point.z = point_out.z
                    self.human_pub.publish(normalized_point)
                    self.get_logger().info(f"ArUco marker detected at coordinates (x: {normalized_point.x}, y: {normalized_point.y}, z: {normalized_point.z})")

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

        img_to_pub = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        img_to_pub.header = data.header
        self.image_out_pub.publish(img_to_pub)

def main(args=None):
    rclpy.init(args=args)
    detect_aruco = DetectAruco()
    rclpy.spin(detect_aruco)
    detect_aruco.destroy_node()
    rclpy.shutdown()