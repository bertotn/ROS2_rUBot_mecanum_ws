import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.5)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        # Periodic cmd_vel publisher at 10 Hz (0.1 s)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        # Wall-following state (for 3s orientation lock)
        self._following_right_since = None
        self._orientation_locked = False

        self.get_logger().info(
            "WallFollower (holonomic) - follows wall on the RIGHT and avoids obstacles."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True

        # Set last command to zero
        self.cmd = Twist()

        # Try a final publish (publisher may still be valid even if shutdown started)
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # Context/publisher may already be invalid -> ignore
            pass

        # Cancel timers safely
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # If the context or publisher is invalid, ignore
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT       = []
        FR_RIGHT    = []
        RIGHT       = []
        BACK_RIGHT  = []
        BACK        = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang <= -160 or ang >= 160:
                BACK.append(d)

        # Minimal distances
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')

        twist = Twist()
        action = ""

        # Choose the sector with the minimum distance
        sectors = {
            'FRONT': min_front,
            'FRONT-RIGHT': min_fr_right,
            'RIGHT': min_right,
            'BACK-RIGHT': min_back_right,
            'BACK': min_back,
        }

        # Find the sector with the smallest measured distance
        sector = min(sectors, key=sectors.get)
        min_val = sectors[sector]

        now_s = self.get_clock().now().nanoseconds * 1e-9

        # Manage following-right timer: track continuous RIGHT visibility
        if sector == 'RIGHT' and math.isfinite(min_right):
            if self._following_right_since is None:
                self._following_right_since = now_s
                self._orientation_locked = False
            elif now_s - self._following_right_since >= 3.0:
                self._orientation_locked = True
        else:
            self._following_right_since = None
            self._orientation_locked = False

        # Safety clamp helper
        def clamp(v, mn, mx):
            return max(mn, min(mx, v))

        # Holonomic motion mapping per user's rules
        # If front obstacle is the closest, move left (over left side)
        if sector == 'FRONT' and math.isfinite(min_val):
            twist.linear.x = 0.0
            twist.linear.y = clamp(self.v_lin * 0.8, -self.v_lin, self.v_lin)
            twist.angular.z = self.v_ang * 0.5
            action = f"FRONT {min_val:.2f} m → move LEFT (avoid)"

        # Front-right: move to front-left (forward + left)
        elif sector == 'FRONT-RIGHT' and math.isfinite(min_val):
            twist.linear.x = clamp(self.v_lin * 0.6, 0.0, self.v_lin)
            twist.linear.y = clamp(self.v_lin * 0.6, -self.v_lin, self.v_lin)
            twist.angular.z = self.v_ang * 0.3
            action = f"FRONT-RIGHT {min_val:.2f} m → move FRONT-LEFT"

        # Right: follow the wall holonomically
        elif sector == 'RIGHT' and math.isfinite(min_val):
            error = min_right - self.base_distance

            # If orientation has been locked for 3s, go forward and keep orientation
            if self._orientation_locked:
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = (
                    f"RIGHT {min_val:.2f} m → FORWARD (orientation locked, parallel)"
                )
            else:
                # Use lateral movement to correct distance to wall (holonomic)
                kp = 0.8
                # linear.y positive = move left; if error>0 (too far) we need to move right (y negative)
                lat = -kp * error
                lat = clamp(lat, -self.v_lin, self.v_lin)

                twist.linear.x = clamp(self.v_lin, 0.0, self.v_lin)
                twist.linear.y = lat
                # small angular damping so robot tends to align while correcting
                twist.angular.z = clamp(-0.5 * error, -self.v_ang, self.v_ang)
                action = (
                    f"RIGHT {min_val:.2f} m (err={error:.2f}) → vx={twist.linear.x:.2f}, vy={twist.linear.y:.2f}"
                )

        # Back-right: move to front-right (forward + right)
        elif sector == 'BACK-RIGHT' and math.isfinite(min_val):
            twist.linear.x = clamp(self.v_lin * 0.6, 0.0, self.v_lin)
            twist.linear.y = clamp(-self.v_lin * 0.4, -self.v_lin, self.v_lin)
            twist.angular.z = -self.v_ang * 0.2
            action = f"BACK-RIGHT {min_val:.2f} m → move FRONT-RIGHT"

        # Back: move over right side (shift right)
        elif sector == 'BACK' and math.isfinite(min_val):
            twist.linear.x = 0.0
            twist.linear.y = clamp(-self.v_lin * 0.8, -self.v_lin, self.v_lin)
            twist.angular.z = -self.v_ang * 0.4
            action = f"BACK {min_val:.2f} m → move RIGHT (shift)"

        else:
            # Nothing visible: stop
            twist = Twist()
            action = "No wall/obstacle detected -> STOP"

        # Emergency: if something very close in front, override to left strafe
        if min_front < 0.3:
            twist.linear.x = 0.0
            twist.linear.y = clamp(self.v_lin * 1.0, -self.v_lin, self.v_lin)
            twist.angular.z = self.v_ang * 0.8
            action += f" | EMERGENCY FRONT CLOSE ({min_front:.2f} m) -> STRAFE LEFT"

        # Update last commanded twist (periodic timer will publish it)
        self.cmd = twist

        # Logging (only on change)
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action (stopped).")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped (no wall detected)"

    #--------------------------------------------------------------------
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
