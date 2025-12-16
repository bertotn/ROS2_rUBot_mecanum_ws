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
        self.declare_parameter('distance_limit', 0.45) # Objectiu: distància a la paret
        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('turn_speed', 0.50)
        self.declare_parameter('time_to_stop', 60.0) # Augmentat per proves
        
        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)

        self.cmd = Twist()

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.info_timer = self.create_timer(0.5, self.log_info) # Log més ràpid
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)
        
        self._state_action = "Idle"
        self._shutting_down = False
        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("WallFollower (Holonomic) - Constant Correction Mode")

    def cmd_publish_timer_cb(self):
        if self._shutting_down:
            return
        self.publisher.publish(self.cmd)

    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)

    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        self.publisher.publish(self.cmd)

    def clamp(self, v, mn, mx):
        return max(mn, min(mx, v))

    def laser_callback(self, scan):
        if self._shutting_down:
            return

        # 1. Processar rangs del làser
        # Definim els angles per als sectors (en graus)
        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)
        
        # Diccionaris per guardar les lectures
        ranges = {
            'FRONT': [],
            'FRONT_RIGHT': [], # Diagonal davant-dreta
            'RIGHT': [],       # Dreta pura (90 graus)
            'BACK_RIGHT': []   # Diagonal darrere-dreta
        }

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d) or d < scan.range_min or d > scan.range_max:
                continue
            
            ang = angle_min + i * angle_inc

            # Definim sectors (ajustar segons el robot)
            if -20 <= ang <= 20:
                ranges['FRONT'].append(d)
            elif -75 <= ang < -20:
                ranges['FRONT_RIGHT'].append(d)
            elif -105 <= ang < -75:
                ranges['RIGHT'].append(d) # Sector clau per la distància
            elif -160 <= ang < -105:
                ranges['BACK_RIGHT'].append(d)

        # Calculem mínims (amb valor segur si està buit)
        mins = {k: min(v) if v else 10.0 for k, v in ranges.items()}
        
        twist = Twist()
        action = ""

        # --- LÒGICA DE CONTROL ---

        # 1. SEGURETAT FRONTAL: Si hi ha obstacle molt a prop al davant
        if mins['FRONT'] < 0.35:
            twist.linear.x = 0.0
            twist.linear.y = self.v_lin  # Desplaçament lateral ESQUERRA (evasió)
            twist.angular.z = self.v_ang # Girar esquerra
            action = f"EMERGENCY FRONT ({mins['FRONT']:.2f}) -> STRAFE LEFT"

        # 2. CANTONADA INTERNA (Mur al davant i a la dreta): Girar i moure's en diagonal
        elif mins['FRONT'] < self.base_distance * 1.5:
            twist.linear.x = 0.05
            twist.linear.y = self.v_lin * 0.8 # Moure esquerra
            twist.angular.z = self.v_ang * 0.8 # Girar esquerra
            action = "CORNER DETECTED -> TURN LEFT + SLIDE"

        # 3. RESEGUIMENT DE PARET (Estat principal)
        # Si detectem paret a la dreta (dins d'un rang raonable, ex: < 1.0m)
        elif mins['RIGHT'] < 1.5 or mins['FRONT_RIGHT'] < 1.0:
            
            # A) CONTROL DISTÀNCIA (Linear Y)
            # Utilitzem principalment el sensor lateral pur ('RIGHT')
            # Si 'RIGHT' no veu res però 'FRONT_RIGHT' sí, usem aquell com a backup
            d_meas = mins['RIGHT']
            if d_meas >= 9.0: # Si RIGHT no veu res
                d_meas = mins['FRONT_RIGHT']

            error = d_meas - self.base_distance

            # Constant Proporcional (KP) més agressiva per mantenir-se enganxat
            kp_lat = 1.5 
            
            # Càlcul velocitat lateral
            # Si error > 0 (lluny) -> lat negatiu (cap a la paret/dreta)
            # Si error < 0 (a prop) -> lat positiu (allunyar-se/esquerra)
            lat = -kp_lat * error
            twist.linear.y = self.clamp(lat, -self.v_lin, self.v_lin)

            # B) CONTROL AVANÇ (Linear X)
            # Si estem molt a prop o l'error és gran, reduïm velocitat d'avanç
            if abs(error) > 0.1:
                twist.linear.x = self.v_lin * 0.5
            else:
                twist.linear.x = self.v_lin

            # C) CONTROL ORIENTACIÓ (Angular Z)
            # Volem estar paral·lels.
            # Heurística: Si Front-Right és més petit que Back-Right, estem "entrant" a la paret -> girar esquerra.
            # Si Front-Right és més gran, estem "sortint" -> girar dreta (però amb cura de no xocar).
            
            w_z = 0.0
            if mins['FRONT_RIGHT'] < 9.0 and mins['BACK_RIGHT'] < 9.0:
                diff = mins['FRONT_RIGHT'] - mins['BACK_RIGHT']
                # Si diff < 0 (davant més a prop que darrere), estem encarats a la paret -> Gir positiu (esquerra)
                # Si diff > 0, estem encarats a fora -> Gir negatiu (dreta) per alinear-se
                kp_rot = 2.0
                w_z = kp_rot * diff 
            else:
                # Si només tenim el sensor lateral, corregim una mica basant-nos en l'error
                # Si estem molt lluny (error gran positiu), girem una mica a la dreta per apropar el morro
                w_z = -0.5 * error 

            twist.angular.z = self.clamp(w_z, -self.v_ang, self.v_ang)
            
            action = f"WALL FOLLOW | Dist: {d_meas:.2f} (Err: {error:.2f}) | Vy: {twist.linear.y:.2f} Wz: {twist.angular.z:.2f}"

        # 4. PARET PERDUDA / RECERCA
        else:
            # Si no veiem res a prop, ens movem a poc a poc endavant i lleugerament a la dreta per buscar la paret
            twist.linear.x = self.v_lin * 0.5
            twist.linear.y = -self.v_lin * 0.3 # Drift a la dreta per trobar paret
            twist.angular.z = 0.0
            action = "SEARCHING WALL -> FORWARD + RIGHT DRIFT"

        self.cmd = twist
        self._state_action = action

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()