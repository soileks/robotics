import rclpy
from rclpy.node import Node
from my_robot_pkg.msg import SensorData

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_sub')
        self.sub = self.create_subscription(
            SensorData, 'sensor_topic', self.cb, 10)
        self.get_logger().info('Sensor Subscriber started')

    def cb(self, msg):
        self.get_logger().info(
            f'Received: {msg.sensor_name}, '
            f'{msg.temperature:.2f}°C, '
            f'id={msg.reading_id}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()