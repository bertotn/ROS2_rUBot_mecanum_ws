import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Paràmetres
        self.declare_parameter('distance_limit', 0.5)    # distància desitjada al mur
        self.declare_parameter('forward_speed', 0.20)    # velocitat "base"
        self.declare_parameter('turn_speed', 0.40)       # (gairebé no l'usem)
        self.declare_parameter('side_speed', 0.18)       # no l'usem directament ara
        self.declare_parameter('time_to_stop', 120.0)    # auto-stop
        self.declare_parameter('tolerance', 0.05)        # tolerància bàsica

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.v_side = float(self.get_parameter('side_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # farem una banda morta una mica més ampla que la tol
        self.dist_deadband = max(0.08, 2.0 * self.tol)

        # Últim comandament
        self.cmd = Twist()

        # Subs i pubs
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("WallFollower HOLONÒMIC (tangent + control suau de distància).")

    # --------------------------------------------------------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    # --------------------------------------------------------------------
    def stop(self):
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
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    # --------------------------------------------------------------------
    def laser_callback(self, scan: LaserScan):
        if self._shutting_down:
            return

        # Busquem el raig amb distància mínima
        min_dist = float('inf')
        min_angle = None

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            if d < min_dist:
                min_dist = d
                min_angle = scan.angle_min + i * scan.angle_increment  # radians

        twist = Twist()
        action = ""

        # Si no veiem res, anem endavant
        if min_angle is None or math.isinf(min_dist):
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = "CAP OBSTACLE DETECTAT -> MOVE FORWARD"

        else:
            # Angle des d'on ve l’obstacle (0 = front, +CCW cap a l’esquerra)
            theta_obs = min_angle

            # Llindars
            desired = self.base_distance   # distància objectiu al mur
            too_close = 0.20               # massa a prop: allunyar-se fort

            # --- Component tangent (el que marca el moviment holonòmic) ---
            # theta_move_tan = theta_obs + 90º -> moviment tangent
            theta_move_tan = theta_obs + math.pi / 2.0
            t_x = math.cos(theta_move_tan)
            t_y = math.sin(theta_move_tan)

            # --- Component radial per controlar distància ---
            error = desired - min_dist     # >0 massa lluny, <0 massa a prop

            if min_dist < too_close:
                # Massa a prop: allunyar-se directament del mur
                theta_move = theta_obs + math.pi      # oposat al mur
                r_x = math.cos(theta_move)
                r_y = math.sin(theta_move)
                v_tan = 0.0                           # de moment, només radial
                v_rad = self.v_lin
                mode = "AWAY (TOO CLOSE)"

            else:
                # Direcció radial segons si hem d'anar cap al mur o allunyar-nos
                if abs(error) < self.dist_deadband:
                    # Som dins de la banda morta -> NO fem correcció radial
                    v_rad = 0.0
                    theta_move_rad = theta_obs  # no importa gaire
                else:
                    # petit guany proporcional
                    k = 0.5
                    v_rad_raw = k * error

                    # Limitem component radial a un % de la tangent
                    v_rad_max = self.v_lin * 0.3
                    if v_rad_raw > v_rad_max:
                        v_rad_raw = v_rad_max
                    elif v_rad_raw < -v_rad_max:
                        v_rad_raw = -v_rad_max

                    v_rad = v_rad_raw

                    # direcció radial: cap al mur o cap enfora
                    if v_rad > 0:
                        theta_move_rad = theta_obs          # cap al mur
                    else:
                        theta_move_rad = theta_obs + math.pi  # cap enfora

                r_x = math.cos(theta_move_rad)
                r_y = math.sin(theta_move_rad)

                # Component tangent constant per seguir la paret
                v_tan = self.v_lin
                mode = "TANGENT+RADIAL"

            # Combinem tangent + radial
            vx = v_tan * t_x + v_rad * r_x
            vy = v_tan * t_y + v_rad * r_y

            # Limitem velocitat total
            v_max = 0.3
            v_norm = math.hypot(vx, vy)
            if v_norm > v_max and v_norm > 1e-6:
                scale = v_max / v_norm
                vx *= scale
                vy *= scale

            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = 0.0

            theta_deg = math.degrees(theta_obs)
            action = (
                f"{mode}: min_dist={min_dist:.2f} m, "
                f"obs_angle={theta_deg:.1f} deg -> "
                f"vx={vx:.2f}, vy={vy:.2f}, error={error:.2f}"
            )

        self.cmd = twist
        self._state_action = action

        if action != self._last_action_logged:
            self.get_logger().info(self._state_action)
            self._last_action_logged = action

    # --------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


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
