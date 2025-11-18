#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan  # Import LaserScan message type
import tf_transformations
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Initialize robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_f = 0.0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Subscribe to odometry and lidar topics
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Create a publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare and get parameters
        self.declare_parameter('vx', 0.3)  # Forward speed
        self.declare_parameter('vy', 0.0)  # Lateral speed (unused in this case)
        self.declare_parameter('w', 0.0)   # Angular velocity
        self.declare_parameter('td', 3.0)  # Time duration for motion
        self.declare_parameter('distance_threshold', 0.5)  # Distance threshold for obstacle

        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.w = self.get_parameter('w').value
        self.td = self.get_parameter('td').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        
        # Start timer to control robot motion
        self.timer = self.create_timer(0.1, self.move_robot)
        
        # Variables for LIDAR processing
        self.lidar_data = None  # This will hold the latest LIDAR scan
        self.min_distance = float('inf')  # Variable to hold the minimum distance

    def odom_callback(self, msg):
        """ Updates robot position from odometry """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_f = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def lidar_callback(self, msg):
        """ Processes LIDAR data to find minimum distance in the forward direction """
        self.lidar_data = msg.ranges  # List of distances from the LIDAR scan
        self.process_lidar_data()

    def process_lidar_data(self):
        """ Finds the minimum distance in the forward direction of the robot """
        if self.lidar_data:
            # Define the angular range for the robot's forward direction
            front_angle_min = -math.pi / 4  # 45 degrees to the left
            front_angle_max = math.pi / 4   # 45 degrees to the right
            min_angle_idx = int((front_angle_min + math.pi) / (2 * math.pi) * len(self.lidar_data))
            max_angle_idx = int((front_angle_max + math.pi) / (2 * math.pi) * len(self.lidar_data))
            
            # Get the minimum distance in the forward direction
            self.min_distance = min(self.lidar_data[min_angle_idx:max_angle_idx])
            
            # Log the minimum distance for debugging
            self.get_logger().info(f'Minimum distance in forward direction: {self.min_distance:.2f} m')

    def move_robot(self):
        """ Publishes velocity commands until time limit is reached """
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        
        if elapsed_time < self.td:
            # Check if the robot is too close to an obstacle
            if self.min_distance < self.distance_threshold:
                self.get_logger().warn(f'Obstacle detected! Stopping robot. Min distance: {self.min_distance:.2f} m')
                self.vx = 0.0  # Stop the robot
            else:
                self.get_logger().info(f'Robot moving... Time elapsed: {elapsed_time:.2f} sec')
            
            vel = Twist()
            vel.linear.x = self.vx
            vel.linear.y = self.vy
            vel.angular.z = self.w
            self.publisher.publish(vel)
        else:
            self.get_logger().warn('Stopping robot')
            self.publisher.publish(Twist())  # Stop the robot
            self.timer.cancel()
            self.get_logger().info("Robot stopped")
            rclpy.try_shutdown()  # Shutdown the node gracefully

def main():
    rclpy.init()
    robot_controller = RobotController()
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()

if __name__ == '__main__':
    main()
