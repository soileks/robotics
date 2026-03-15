import rclpy
import math
from rclpy.node import Node
from my_robot_pkg.srv import ComputeDistance

def main():
    rclpy.init()
    node = Node('dist_server')
    
    def handle(req, resp):
        dx = req.x2 - req.x1
        dy = req.y2 - req.y1
        resp.distance = math.sqrt(dx**2 + dy**2)
        node.get_logger().info(f'Distance: {resp.distance:.2f}')
        return resp
    
    node.create_service(ComputeDistance, 'compute_dist', handle)
    node.get_logger().info('Server ready')
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()