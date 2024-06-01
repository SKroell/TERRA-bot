import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class DriveSquare(Node):

    def __init__(self):
        super().__init__('drive_square')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Distance and speed parameters
        self.drive_distance = 2.4  # meters
        self.turn_angle = 90  # degrees
        self.max_linear_speed = 0.7  # meters per second
        self.max_angular_speed = 0.5  # radians per second

        # Ramp-up parameters
        self.linear_acceleration = 0.1  # meters per second^2
        self.angular_acceleration = 0.2   # radians per second^2
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.distance_covered = 0.0
        self.angle_covered = 0.0

        self.state = 0
        self.state_start_time = time.time()
        self.sides_completed = 0
        self.start_time = time.time()  # Start timing

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def ramp_speed(self, target_speed, current_speed, acceleration):
        if current_speed < target_speed:
            return min(current_speed + acceleration * 0.1, target_speed)
        else:
            return target_speed

    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.sides_completed < 100:
            if self.state == 0:  # Drive forward
                if self.distance_covered < self.drive_distance:
                    self.current_linear_speed = self.ramp_speed(self.max_linear_speed, self.current_linear_speed, self.linear_acceleration)
                    msg.twist.linear.x = self.current_linear_speed
                    self.distance_covered += self.current_linear_speed * 0.1  # distance = speed * time
                else:
                    self.state = 1
                    self.state_start_time = current_time
                    self.distance_covered = 0.0
                    self.current_linear_speed = 0.0  # Reset speed for the next phase

            elif self.state == 1:  # Turn right
                if self.angle_covered < self.turn_angle * (3.14159 / 180):
                    self.current_angular_speed = self.ramp_speed(self.max_angular_speed, self.current_angular_speed, self.angular_acceleration)
                    msg.twist.angular.z = self.current_angular_speed
                    self.angle_covered += self.current_angular_speed * 0.1  # angle = angular speed * time
                else:
                    self.state = 0
                    self.state_start_time = current_time
                    self.angle_covered = 0.0
                    self.sides_completed += 1
                    self.current_angular_speed = 0.0  # Reset speed for the next phase
        else:
            self.get_logger().info(f"Completed the square in {current_time - self.start_time:.2f} seconds")
            rclpy.shutdown()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    drive_square = DriveSquare()
    rclpy.spin(drive_square)
    drive_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
