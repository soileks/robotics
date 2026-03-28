import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.sub = self.create_subscription(
            Pose, '/turtle1/pose',
            self.on_pose, 10)

    def on_pose(self, msg):
        self.get_logger().info(
            f'x={msg.x:.2f} y={msg.y:.2f} theta={msg.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
