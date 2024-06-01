import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco

class DetectAruco(Node):

    def __init__(self):
        super().__init__('detect_aruco')

        self.get_logger().info('Looking for an ArUco marker...')
        self.image_sub = self.create_subscription(Image, "/image_in", self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.aruco_pub = self.create_publisher(Point, "/detected_aruco", 1)

        self.bridge = CvBridge()

        # Load the predefined dictionary
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        try:
            # Convert to grayscale for better detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect the markers in the image
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                # Draw detected markers
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # Find the center of the first detected marker
                point_out = Point()

                for corner in corners:
                    corner = corner[0]
                    center_x = int((corner[0][0] + corner[2][0]) / 2)
                    center_y = int((corner[0][1] + corner[2][1]) / 2)
                    point_out.x = center_x
                    point_out.y = center_y

                    # Calculate the area of the marker
                    marker_width = corner[2][0] - corner[0][0]
                    marker_height = corner[2][1] - corner[0][1]
                    marker_area = marker_width * marker_height

                    # Calculate the area of the image
                    image_height, image_width = cv_image.shape[:2]
                    image_area = image_width * image_height

                    # Normalize the area to get the z value
                    point_out.z = marker_area / image_area

                    # Draw the center
                    cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    # Publish the first detected marker's center and size
                    normalized_point = self.normalize_coordinates(cv_image, point_out.x, point_out.y)
                    normalized_point.z = point_out.z
                    self.aruco_pub.publish(normalized_point)
                    self.get_logger().info(f"ArUco marker detected at normalized coordinates (x: {normalized_point.x}, y: {normalized_point.y}), z: {normalized_point.z}")
                    break  # Only process the first detected marker

            img_to_pub = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def normalize_coordinates(self, image, x, y):
        height, width, _ = image.shape
        normalized_x = (x - (width / 2)) / (width / 2)
        normalized_y = (y - (height / 2)) / (height / 2)
        return Point(x=normalized_x, y=normalized_y, z=1.0)

def main(args=None):
    rclpy.init(args=args)
    detect_aruco = DetectAruco()
    rclpy.spin(detect_aruco)
    detect_aruco.destroy_node()
    rclpy.shutdown()
