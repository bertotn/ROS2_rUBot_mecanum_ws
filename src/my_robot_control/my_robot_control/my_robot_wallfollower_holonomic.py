#!/usr/bin/env python3
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
        self.declare_parameter('distance_limit', 0.5)    # distància "massa a prop"
        self.declare_parameter('forward_speed', 0.20)    # mòdul de velocitat lineal
        self.declare_parameter('turn_speed', 0.40)       # per si vols girs (no essencial aquí)
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # no s'utilitza gaire ara, però el deixem

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Últim comandament Twist (es publicarà periòdicament)
        self.cmd = Twist()

        # Entitats ROS 2
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
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
            "WallFollower – HOLONOMIC (regles per sectors)."
        )

    # -------------------------------------------------------------------
    def stop_watchdog(self):
        """Atura el robot després de time_to_stop segons."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    # -------------------------------------------------------------------
    def stop(self):
        """Aturada segura: cmd = 0, publicar un cop i cancel·lar timers."""
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

    # -------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Publicador periòdic: envia el darrer cmd_vel a 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    # -------------------------------------------------------------------
    def laser_callback(self, scan: LaserScan):
        """Control holonòmic amb regles de prioritat segons el sector."""
        if self._shutting_down:
            return

        angle_min_deg = math.degrees(scan.angle_min)
        angle_inc_deg = math.degrees(scan.angle_increment)

        FRONT       = []
        FR_RIGHT    = []
        RIGHT       = []
        BR_RIGHT    = []
        BACK        = []

        # Classifiquem els punts del LIDAR en sectors angulars
        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min_deg + i * angle_inc_deg  # en graus

            # Ajusta aquests rangs si cal segons la teva configuració
            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BR_RIGHT.append(d)
            elif ang <= -160 or ang >= 160:
                BACK.append(d)

        # Distàncies mínimes per sector
        min_front = min(FRONT)    if FRONT    else float('inf')
        min_fr    = min(FR_RIGHT) if FR_RIGHT else float('inf')
        min_right = min(RIGHT)    if RIGHT    else float('inf')
        min_br    = min(BR_RIGHT) if BR_RIGHT else float('inf')
        min_back  = min(BACK)     if BACK     else float('inf')

        twist = Twist()
        action = ""

        # Llindar de "massa a prop"
        d_lim = self.base_distance

        # ---------------- Regles amb prioritat ----------------
        # 1) FRONT
        if min_front < d_lim:
            # FRONT → moure pel costat esquerre (només lateral)
            twist.linear.x = 0.0
            twist.linear.y = self.v_lin          # +y = esquerra
            twist.angular.z = 0.0
            action = f"FRONT {min_front:.2f} m → move LEFT"

        # 2) FRONT-RIGHT
        elif min_fr < d_lim:
            # FRONT-RIGHT → moure cap al front-esquerra (diagonal)
            twist.linear.x = self.v_lin * 0.7    # endavant
            twist.linear.y = self.v_lin * 0.7    # esquerra
            twist.angular.z = 0.0
            action = f"FRONT-RIGHT {min_fr:.2f} m → move FRONT-LEFT"

        # 3) RIGHT
        elif min_right < d_lim:
            # RIGHT → avançar endavant i mantenir orientació paral·lela a la paret
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = f"RIGHT {min_right:.2f} m → move FORWARD"

        # 4) BACK-RIGHT
        elif min_br < d_lim:
            # BACK-RIGHT → moure cap a front-dreta (diagonal)
            twist.linear.x = self.v_lin * 0.7    # endavant
            twist.linear.y = -self.v_lin * 0.7   # dreta
            twist.angular.z = 0.0
            action = f"BACK-RIGHT {min_br:.2f} m → move FRONT-RIGHT"

        # 5) BACK
        elif min_back < d_lim:
            # BACK → moure pel costat dret (només lateral)
            twist.linear.x = 0.0
            twist.linear.y = -self.v_lin         # -y = dreta
            twist.angular.z = 0.0
            action = f"BACK {min_back:.2f} m → move RIGHT"

        else:
            # Si no hi ha cap paret prou a prop, aturem el robot
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            action = "No close wall → STOP"

        # Guardem el comandament perquè el publiqui el timer
        self.cmd = twist

        # Només fem log si canvia l'acció per no saturar la consola
        if action != self._last_action_logged:
            self.get_logger().info(action)
            self._last_action_logged = action

        self._state_action = action

    # -------------------------------------------------------------------
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
