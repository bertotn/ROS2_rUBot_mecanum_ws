import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class WallFollowerHolonomic(Node):
    def __init__(self):
        super().__init__('wall_follower_holonomic_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.5)    # desired distance to walls
        self.declare_parameter('forward_speed', 0.20)    # nominal forward speed
        self.declare_parameter('strafe_speed', 0.20)     # sideways speed (|vy|)
        self.declare_parameter('turn_speed', 0.40)       # max angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # not used for holonomic rules now
        self.declare_parameter('k_ang', 1.0)             # proportional gain for yaw control

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_strafe = float(self.get_parameter('strafe_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)
        self.k_ang = float(self.get_parameter('k_ang').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # Odometry / orientation
        self.current_yaw = 0.0
        self.odom_received = False

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)  # 10 Hz

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower HOLONOMIC – vx, vy, w with full left/right handling and yaw alignment."
        )

    # --------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        """Read current yaw from /odom."""
        q = msg.pose.pose.orientation
        # Yaw from quaternion (z-w)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    # --------------------------------------------------------------------
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # --------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    # --------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True

        self.cmd = Twist()

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    # --------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    # --------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # Right side regions (negative angles)
        FRONT        = []
        FR_RIGHT     = []
        RIGHT        = []
        BACK_RIGHT   = []
        BACK         = []

        # Left side regions (positive angles)
        FR_LEFT      = []
        LEFT         = []
        BACK_LEFT    = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            # Right side
            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang <= -160:
                BACK.append(d)

            # Left side (mirror)
            elif 20 < ang <= 70:
                FR_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)
            elif 110 < ang <= 160:
                BACK_LEFT.append(d)
            elif ang >= 160:
                BACK.append(d)  # also behind

        # Minimal distances per region
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')
        min_fr_left    = min(FR_LEFT)    if FR_LEFT    else float('inf')
        min_left       = min(LEFT)       if LEFT       else float('inf')
        min_back_left  = min(BACK_LEFT)  if BACK_LEFT  else float('inf')

        twist = Twist()
        action = ""

        # Dictionary of all regions and their minima
        region_mins = {
            "FRONT":       min_front,
            "FR_RIGHT":    min_fr_right,
            "RIGHT":       min_right,
            "BACK_RIGHT":  min_back_right,
            "BACK":        min_back,
            "FR_LEFT":     min_fr_left,
            "LEFT":        min_left,
            "BACK_LEFT":   min_back_left,
        }

        # Find region with the smallest distance
        closest_region = min(region_mins, key=region_mins.get)
        closest_dist = region_mins[closest_region]

        # --- 1) Decide ONLY linear velocities segons la paret ---
        if closest_dist == float('inf') or closest_dist > self.base_distance:
            # Nothing close → go forward
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            action = "No close wall → move FORWARD"
        else:
            if closest_region == "FRONT":
                # Paret davant → mou-te cap a l'esquerra
                twist.linear.x = 0.0
                twist.linear.y = self.v_strafe
                action = f"FRONT {closest_dist:.2f} m → move LEFT (vy>0)"

            elif closest_region == "FR_RIGHT":
                # Paret front-dreta → mou-te front-esquerra
                twist.linear.x = self.v_lin
                twist.linear.y = self.v_strafe
                action = f"FRONT-RIGHT {closest_dist:.2f} m → move FRONT-LEFT"

            elif closest_region == "RIGHT":
                # Paret dreta → mou-te endavant (seguint la paret a la dreta)
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = f"RIGHT {closest_dist:.2f} m → move FORWARD (parallel to right wall)"

            elif closest_region == "BACK_RIGHT":
                # Paret darrere-dreta → mou-te front-dreta
                twist.linear.x = self.v_lin
                twist.linear.y = -self.v_strafe
                action = f"BACK-RIGHT {closest_dist:.2f} m → move FRONT-RIGHT"

            elif closest_region == "BACK":
                # Paret darrere → mou-te cap a la dreta
                twist.linear.x = 0.0
                twist.linear.y = -self.v_strafe
                action = f"BACK {closest_dist:.2f} m → move RIGHT (vy<0)"

            elif closest_region == "FR_LEFT":
                # Paret front-esquerra → mou-te BACK-LEFT (exemple)
                twist.linear.x = -self.v_lin
                twist.linear.y = self.v_strafe
                action = f"FRONT-LEFT {closest_dist:.2f} m → move BACK-LEFT"

            elif closest_region == "LEFT":
                # Paret esquerra → mou-te enrere (o com vulguis)
                twist.linear.x = -self.v_lin
                twist.linear.y = 0.0
                action = f"LEFT {closest_dist:.2f} m → move BACKWARD"

            elif closest_region == "BACK_LEFT":
                # Paret darrere-esquerra → mou-te BACK-RIGHT
                twist.linear.x = -self.v_lin
                twist.linear.y = -self.v_strafe
                action = f"BACK-LEFT {closest_dist:.2f} m → move BACK-RIGHT"

        # --- 2) Orientació: fer que el robot miri cap a la direcció de moviment ---
        vx = twist.linear.x
        vy = twist.linear.y

        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            # Si no hi ha moviment → no girem
            twist.angular.z = 0.0
        else:
            # Angle de la direcció de moviment en l'eix del robot
            desired_yaw = math.atan2(vy, vx)   # ex: vx=0, vy>0 → +90º

            if self.odom_received:
                yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)
                w_cmd = self.k_ang * yaw_error

                # Limitem la velocitat angular màxima
                if w_cmd > self.v_ang:
                    w_cmd = self.v_ang
                elif w_cmd < -self.v_ang:
                    w_cmd = -self.v_ang

                twist.angular.z = w_cmd
            else:
                # Encara no tenim /odom → no girem per seguretat
                twist.angular.z = 0.0

        # Update last commanded twist
        self.cmd = twist

        # Logging (only on change)
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action (stopped).")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped (no wall detected)"

    # --------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerHolonomic()
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

