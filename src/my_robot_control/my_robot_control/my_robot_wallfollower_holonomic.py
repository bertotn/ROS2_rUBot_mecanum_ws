import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower_node')

        # -------------------- Parameters --------------------
        self.declare_parameter('distance_limit', 0.25)   # desired distance to right wall
        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('time_to_stop', 30.0)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)

        # Controller gains (augmentar control lateral i angular)
        self.kp_ang = 1.5    # angular correction (MAIN) -> augmentat
        self.kp_lat = 0.8    # lateral correction (major) -> augmentat

        # ROS entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.cmd_timer = self.create_timer(0.1, self.publish_cmd)
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        # State
        self.cmd = Twist()
        self._state_action = "Idle"
        self._shutting_down = False
        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("WallFollower holonòmic — seguiment paret DRETA")


    # ------------------------------------------------------
    def clamp(self, v, mn, mx):
        return max(mn, min(mx, v))


    # ------------------------------------------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Temps esgotat → STOP")
            self.stop()


    # ------------------------------------------------------
    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

        for t in [self.cmd_timer, self.info_timer, self.stop_timer]:
            try:
                t.cancel()
            except Exception:
                pass


    # ------------------------------------------------------
    def publish_cmd(self):
        if not self._shutting_down:
            self.publisher.publish(self.cmd)


    # ------------------------------------------------------
    def laser_callback(self, scan):

        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT = []
        RIGHT = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            # sectors
            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -110 <= ang <= -70:
                RIGHT.append(d)

        min_front = min(FRONT) if FRONT else float('inf')
        min_right = min(RIGHT) if RIGHT else float('inf')

        twist = Twist()

        # ---------------- EMERGENCY FRONT ----------------
        if min_front < 0.30:
            twist.linear.x = 0.0
            twist.linear.y = self.v_lin
            twist.angular.z = self.v_ang
            self.cmd = twist
            self._state_action = f"EMERGENCY FRONT ({min_front:.2f} m)"
            return

        # ---------------- WALL FOLLOW RIGHT ----------------
        if min_right < float('inf'):

            error = self.base_distance - min_right
            ang_corr = self.clamp(self.kp_ang * error, -self.v_ang, self.v_ang)
            lat_corr = self.clamp(self.kp_lat * error, -0.1, 0.1)

            twist.linear.x = self.v_lin
            twist.linear.y = lat_corr
            twist.angular.z = ang_corr

            self._state_action = (
                f"FOLLOW RIGHT | d={min_right:.2f} err={error:.2f}"
            )

        # ---------------- NO WALL ----------------
        else:
            twist.linear.x = 0.1
            twist.angular.z = -0.2
            self._state_action = "Searching for wall → rotate right"

        self.cmd = twist


    # ------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


# ----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
