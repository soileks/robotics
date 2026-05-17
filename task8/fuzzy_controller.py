import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path

def tri(x, a, b, c):
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    return (x - a) / (b - a) if x < b else (c - x) / (c - b)

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.path_points = []
        self.target_idx = 0
        self.path_received = False
        
        self.get_logger().info('Fuzzy Controller Started (Mamdani)')
    
    def path_cb(self, msg):
        if len(msg.poses) > 0:
            self.path_points = msg.poses
            self.target_idx = 0
            self.path_received = True
            self.get_logger().info(f'Path received: {len(self.path_points)} points')
    
    def sector_min(self, scan, a_min, a_max):
        vals = []
        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i * scan.angle_increment
            ang = (ang + math.pi) % (2 * math.pi) - math.pi
            if a_min <= ang <= a_max and math.isfinite(r) and r > 0.0:
                vals.append(r)
        return min(vals) if vals else 1.5
    
    def fuzzify_distance(self, d):
        return {
            'close':  tri(d, 0.0, 0.0, 0.7),
            'medium': tri(d, 0.3, 0.7, 1.2),
            'far':    tri(d, 0.8, 1.5, 3.0)
        }
    
    def scan_cb(self, scan):
        d_front = self.sector_min(scan, -0.35, 0.35)
        d_left = self.sector_min(scan, 0.35, 1.40)
        d_right = self.sector_min(scan, -1.40, -0.35)
        
        if not self.path_received or self.target_idx >= len(self.path_points):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return
        
        target = self.path_points[self.target_idx].pose.position
        dx = target.x
        dy = target.y
        dist_to_target = math.hypot(dx, dy)
        
        if dist_to_target < 0.25:
            self.target_idx += 1
            if self.target_idx >= len(self.path_points):
                self.path_received = False
                self.get_logger().info('Goal reached!')
                return
            target = self.path_points[self.target_idx].pose.position
            dx = target.x
            dy = target.y
        
        target_angle = math.atan2(dy, dx)
        
        F = self.fuzzify_distance(d_front)
        L = self.fuzzify_distance(d_left)
        R = self.fuzzify_distance(d_right)
        
        w1 = F['close']
        w2 = min(F['medium'], R['close'])
        w3 = min(F['far'], 1 - R['close'])
        w4 = min(F['far'], L['close'])
        
        speed_rules = [(w1, 0.08), (w2, 0.15), (w3, 0.22), (w4, 0.15)]
        turn_rules = [(w1, 0.5), (w2, 0.3), (w3, -0.2), (w4, -0.3)]

        speed = sum(w * s for w, s in speed_rules) / (sum(w for w, _ in speed_rules) + 0.001)
        turn = sum(w * t for w, t in turn_rules) / (sum(w for w, _ in turn_rules) + 0.001)
 
        turn = turn * 0.6 + target_angle * 0.4

        speed = max(0.05, min(0.25, speed))
        turn = max(-0.5, min(0.5, turn))

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = speed
        msg.twist.angular.z = turn
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = FuzzyController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
