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

        # Parameters (velocitats reduïdes)
        self.declare_parameter('distance_limit', 0.5)    # distància desitjada a la paret
        self.declare_parameter('forward_speed', 0.10)    # velocitat nominal en x
        self.declare_parameter('strafe_speed', 0.10)     # velocitat lateral |vy|
        self.declare_parameter('turn_speed', 0.25)       # velocitat angular màxima
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('k_ang', 1.0)             # guany proporcional del control de yaw

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

        # Estat per detectar paret "estable"
        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9
        self.stable_wall_time = 2.0  # segons que ha d'estar a la mateixa paret
        self.last_region = None
        self.last_region_change_time = self.start_time_s
        self.wall_align_active = False
        self.target_yaw = 0.0  # orientació objectiu quan decidim alinear-nos amb la paret

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

        self.get_logger().info(
            "WallFollower HOLONOMIC – vx, vy, w amb alineació de yaw i velocitats reduïdes."
        )

    # --------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        """Read current yaw from /odom."""
        q = msg.pose.pose.orientation
        # Yaw from quaternion
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
    def compute_target_yaw_from_region(self, region: str) -> float:
        """
        Defineix quina orientació volem quan portem un temps amb una paret concreta.
        Ara mateix:
        - Paret a la dreta o variants → yaw = 0.0 (anar endavant, paret al costat)
        - Paret a l'esquerra → també 0.0
        - Paret darrere → pi (girar 180º)
        Pots canviar-ho fàcilment (p. ex. RIGHT → math.pi/2.0 si vols 90º).
        """
        if region in ["RIGHT", "FR_RIGHT", "BACK_RIGHT"]:
            return 0.0
        elif region in ["LEFT", "FR_LEFT", "BACK_LEFT"]:
            return 0.0
        elif region == "BACK":
            return math.pi
        else:
            # FRONT o sense res especial → mantenim 0
            return 0.0

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

            # Right side (angles negatius o propers al davant)
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

            # Left side
            elif 20 < ang <= 70:
                FR_LEFT.append(d)
            elif 70 < ang <= 110:
                LEFT.append(d)
            elif 110 < ang <= 160:
                BACK_LEFT.append(d)
            elif ang >= 160:
                BACK.append(d)  # també darrere

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

        # --- Detectar si portem temps amb la mateixa paret ---
        now = self.get_clock().now().nanoseconds * 1e-9

        if closest_region != self.last_region:
            # Ha canviat la paret més propera → reiniciem comptador
            self.last_region = closest_region
            self.last_region_change_time = now
            self.wall_align_active = False
        else:
            # Mateixa regió que abans
            time_same_wall = now - self.last_region_change_time
            if (closest_dist < float('inf') and
                time_same_wall > self.stable_wall_time and
                not self.wall_align_active):

                # Activem mode "alineat amb la paret"
                self.wall_align_active = True
                self.target_yaw = self.compute_target_yaw_from_region(closest_region)
                self.get_logger().info(
                    f"Wall {closest_region} estable durant {time_same_wall:.1f}s → "
                    f"alineant orientació (target_yaw={self.target_yaw:.2f} rad)"
                )

        # --- 1) Decidir només velocitats lineals segons la paret ---
        if closest_dist == float('inf') or closest_dist > self.base_distance:
            # Res a prop → endavant
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
                # Paret front-dreta → front-esquerra
                twist.linear.x = self.v_lin
                twist.linear.y = self.v_strafe
                action = f"FRONT-RIGHT {closest_dist:.2f} m → move FRONT-LEFT"

            elif closest_region == "RIGHT":
                # Paret dreta → endavant
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = f"RIGHT {closest_dist:.2f} m → move FORWARD (parallel to right wall)"

            elif closest_region == "BACK_RIGHT":
                # Paret darrere-dreta → front-dreta
                twist.linear.x = self.v_lin
                twist.linear.y = -self.v_strafe
                action = f"BACK-RIGHT {closest_dist:.2f} m → move FRONT-RIGHT"

            elif closest_region == "BACK":
                # Paret darrere → cap a la dreta
                twist.linear.x = 0.0
                twist.linear.y = -self.v_strafe
                action = f"BACK {closest_dist:.2f} m → move RIGHT (vy<0)"

            elif closest_region == "FR_LEFT":
                # Paret front-esquerra → BACK-LEFT (per exemple)
                twist.linear.x = -self.v_lin
                twist.linear.y = self.v_strafe
                action = f"FRONT-LEFT {closest_dist:.2f} m → move BACK-LEFT"

            elif closest_region == "LEFT":
                # Paret esquerra → enrere
                twist.linear.x = -self.v_lin
                twist.linear.y = 0.0
                action = f"LEFT {closest_dist:.2f} m → move BACKWARD"

            elif closest_region == "BACK_LEFT":
                # Paret darrere-esquerra → BACK-RIGHT
                twist.linear.x = -self.v_lin
                twist.linear.y = -self.v_strafe
                action = f"BACK-LEFT {closest_dist:.2f} m → move BACK-RIGHT"

        # --- 2) Orientació: yaw segons moviment o segons paret estable ---
        vx = twist.linear.x
        vy = twist.linear.y

        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            # Si no ens movem → no girem
            twist.angular.z = 0.0
        else:
            # Si ja hem detectat una paret "estable", fem servir target_yaw
            if self.wall_align_active:
                desired_yaw = self.target_yaw
            else:
                # Si no, intentem mirar cap a on ens estem movent
                desired_yaw = math.atan2(vy, vx)

            if self.odom_received:
                yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)
                w_cmd = self.k_ang * yaw_error

                # Limitar velocitat angular
                if w_cmd > self.v_ang:
                    w_cmd = self.v_ang
                elif w_cmd < -self.v_ang:
                    w_cmd = -self.v_ang

                twist.angular.z = w_cmd
            else:
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

