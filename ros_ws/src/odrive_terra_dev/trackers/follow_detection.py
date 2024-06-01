import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import time

class FollowHuman(Node):

    def __init__(self):
        super().__init__('follow_human')
        self.subscription = self.create_subscription(
            Point,
            '/detected_human',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.2)
        self.declare_parameter("search_angular_speed", 0.3)
        self.declare_parameter("max_size_thresh", 0.9)
        self.declare_parameter("min_distance_thresh", 0.002)  # New parameter
        self.declare_parameter("filter_value", 0.9)
        self.declare_parameter("lost_target_delay_secs", 2.0)
        self.declare_parameter("lost_target_angular_speed", 0.1)
        self.declare_parameter("target_acquisition_time_secs", 2.0)

        self.lost_target_angular_speed = self.get_parameter('lost_target_angular_speed').get_parameter_value().double_value
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.min_distance_thresh = self.get_parameter('min_distance_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        self.lost_target_delay_secs = self.get_parameter('lost_target_delay_secs').get_parameter_value().double_value
        self.target_acquisition_time_secs = self.get_parameter('target_acquisition_time_secs').get_parameter_value().double_value

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000
        self.lost_target_time = None
        self.target_acquired_time = None  # to store the time when the target was first acquired
        self.last_known_direction = 0  # 0 for left, 1 for right

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        if (current_time - self.lastrcvtime < self.rcv_timeout_secs):
            if self.target_acquired_time is None:
                self.target_acquired_time = current_time  # mark the time when the target was first acquired
            elif current_time - self.target_acquired_time > self.target_acquisition_time_secs:
                # Only follow the target if it has been acquired for more than target_acquisition_time_secs
                self.get_logger().info('Target: {}'.format(self.target_val))
                print(f'{self.min_distance_thresh} < {self.target_dist} < {self.max_size_thresh}')
                if self.min_distance_thresh < self.target_dist < self.max_size_thresh:
                    msg.linear.x = self.forward_chase_speed
                msg.angular.z = self.angular_chase_multiplier * self.target_val
                self.lost_target_time = None  # reset the lost target time when the target is found
                # Store the last known direction
                self.last_known_direction = 0 if self.target_val < 0 else 1
        else:
            if self.lost_target_time is None:
                self.lost_target_time = current_time  # store the time when the target was lost
            elif current_time - self.lost_target_time < self.lost_target_delay_secs:
                # Continue moving towards the last known target position
                if self.min_distance_thresh < self.target_dist < self.max_size_thresh:
                    msg.linear.x = self.forward_chase_speed
                msg.angular.z = self.angular_chase_multiplier * self.target_val
            else:
                self.get_logger().info('Target lost')
                # Rotate in the last known direction
                msg.angular.z = self.lost_target_angular_speed if self.last_known_direction == 0 else -self.lost_target_angular_speed

        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1 - f)
        self.target_dist = self.target_dist * f + msg.z * (1 - f)
        self.lastrcvtime = time.time()

def main(args=None):
    rclpy.init(args=args)
    follow_human = FollowHuman()
    rclpy.spin(follow_human)
    follow_human.destroy_node()
    rclpy.shutdown()
