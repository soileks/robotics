import heapq
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        self.map_msg = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal', self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
        self.get_logger().info('A* Path Planner Started')
    
    def map_cb(self, msg):
        self.map_msg = msg
    
    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
    
    def goal_cb(self, msg):
        if self.map_msg is None:
            self.get_logger().warn('No map yet')
            return
        self.plan_and_publish(msg.pose.position.x, msg.pose.position.y)
    
    def world_to_grid(self, x, y):
        info = self.map_msg.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        info = self.map_msg.info
        x = gx * info.resolution + info.origin.position.x
        y = gy * info.resolution + info.origin.position.y
        return x, y
    
    def is_free(self, gx, gy):
        info = self.map_msg.info
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return False
        v = self.map_msg.data[gy * info.width + gx]
        return v == -1 or (v >= 0 and v < 50)
    
    def neighbors(self, cell):
        gx, gy = cell
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]:
            nx, ny = gx + dx, gy + dy
            if self.is_free(nx, ny):
                yield (nx, ny)
    
    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
    
    def a_star(self, start, goal):
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if abs(current[0] - goal[0]) < 2 and abs(current[1] - goal[1]) < 2:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))
            
            for nb in self.neighbors(current):
                cost = 1.0
                if abs(nb[0] - current[0]) == 1 and abs(nb[1] - current[1]) == 1:
                    cost = 1.414
                tentative = g_score[current] + cost
                if tentative < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = tentative
                    f = tentative + self.heuristic(nb, goal)
                    heapq.heappush(open_set, (f, nb))
        return []
    
    def plan_and_publish(self, goal_x, goal_y):
        start = self.world_to_grid(self.robot_x, self.robot_y)
        goal = self.world_to_grid(goal_x, goal_y)
        
        self.get_logger().info(f'Robot pos: ({self.robot_x:.2f}, {self.robot_y:.2f}) -> grid {start}')
        self.get_logger().info(f'Goal: ({goal_x:.2f}, {goal_y:.2f}) -> grid {goal}')
        
        if not self.is_free(*start):
            self.get_logger().warn('Start cell is not free')
            return
        if not self.is_free(*goal):
            self.get_logger().warn('Goal cell is not free, but planning anyway')
        
        cells = self.a_star(start, goal)
        if not cells:
            self.get_logger().warn('No path found')
            return
        
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for c in cells:
            wx, wy = self.grid_to_world(*c)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        
        self.path_pub.publish(path)
        self.get_logger().info(f'Path found with {len(cells)} points')

def main():
    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()