import rclpy
import random
from rclpy.node import Node
from my_robot_pkg.msg import SensorData

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_pub')
        self.pub = self.create_publisher(SensorData, 'sensor_topic', 10)
        self.timer = self.create_timer(0.5, self.cb)
        self.get_logger().info('Sensor Publisher started')

    def cb(self):
        msg = SensorData()
        msg.sensor_name = 'temp_01'
        msg.temperature = random.uniform(20.0, 45.0)
        msg.reading_id = random.randint(1, 1000)
        msg.is_active = True
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {msg.temperature:.2f}°C, id={msg.reading_id}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()