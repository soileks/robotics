import rclpy
from rclpy.node import Node
from my_robot_pkg.srv import ComputeDistance

class DistanceClient(Node):
    def __init__(self):
        super().__init__('dist_client')
        self.cli = self.create_client(ComputeDistance, 'compute_dist')

    def send_request(self):
        req = ComputeDistance.Request()
        req.x1, req.y1 = 0.0, 0.0
        req.x2, req.y2 = 3.0, 4.0

        future = self.cli.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Distance: {future.result().distance:.2f} (ожидалось 5.00)')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = DistanceClient()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()