import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        self.resolution = 0.05
        self.width = 200
        self.height = 200
        self.origin_x = -5.0
        self.origin_y = -5.0

        self.l_occ = 0.85
        self.l_free = -0.4
        self.l_min = -5.0
        self.l_max = 5.0

        self.log_odds = np.zeros((self.height, self.width))
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_map)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('SLAM Node Started')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return None
        return (gx, gy)

    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def update_cell(self, gx, gy, value):
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return
        self.log_odds[gy, gx] += value
        if self.log_odds[gy, gx] < self.l_min:
            self.log_odds[gy, gx] = self.l_min
        elif self.log_odds[gy, gx] > self.l_max:
            self.log_odds[gy, gx] = self.l_max

    def scan_callback(self, msg):
        robot_grid = self.world_to_grid(self.robot_x, self.robot_y)
        if robot_grid is None:
            return

        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment + self.robot_theta
            end_x = self.robot_x + r * math.cos(angle)
            end_y = self.robot_y + r * math.sin(angle)

            end_grid = self.world_to_grid(end_x, end_y)
            if end_grid is None:
                continue

            cells = self.bresenham(robot_grid[0], robot_grid[1], end_grid[0], end_grid[1])

            for cell in cells[:-1]:
                self.update_cell(cell[0], cell[1], self.l_free)

            if len(cells) > 0:
                self.update_cell(cells[-1][0], cells[-1][1], self.l_occ)

    def publish_map(self):
        map_data = np.zeros((self.height, self.width), dtype=np.int8)

        for y in range(self.height):
            for x in range(self.width):
                log_odd = self.log_odds[y, x]
                if log_odd == 0:
                    map_data[y, x] = -1
                else:
                    prob = 1.0 - 1.0 / (1.0 + math.exp(log_odd))
                    map_data[y, x] = int(prob * 100)

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.data = map_data.flatten().tolist()

        self.map_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()