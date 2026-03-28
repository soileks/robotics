import rclpy
from rclpy.node import Node
from turtle_controller_pkg.srv import SpawnTurtle
import random
import string

class SpawnClient(Node):
    def __init__(self):
        super().__init__('spawn_client')
        self.cli = self.create_client(SpawnTurtle, 'spawn_turtle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn_turtle service...')
        self.future = None

    def send_request(self, name, x, y):
        req = SpawnTurtle.Request()
        req.turtle_name = name
        req.x = x
        req.y = y
        self.future = self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = SpawnClient()
    suffix = ''.join(random.choices(string.ascii_lowercase, k=4))
    name = f'turtle_{suffix}'
    x = round(random.uniform(1.0, 10.0), 1)
    y = round(random.uniform(1.0, 10.0), 1)
    node.send_request(name, x, y)
    rclpy.spin_until_future_complete(node, node.future)
    result = node.future.result()
    node.get_logger().info(f'{result.message}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
