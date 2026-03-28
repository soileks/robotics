import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move)

        self.forward_duration = 2.0
        self.turn_duration = 0.87

        self.state = 'forward'
        self.side_count = 0
        self.time_in_state = 0

    def move(self):
        msg = Twist()
        self.time_in_state += 0.1

        if self.state == 'forward':
            msg.linear.x = 2.0
            msg.angular.z = 0.0

            if self.time_in_state >= self.forward_duration:
                self.state = 'turn'
                self.time_in_state = 0
                self.side_count += 1
                self.get_logger().info(f'Поворачиваю... Сторона {self.side_count}')

        elif self.state == 'turn':
            msg.linear.x = 0.0
            msg.angular.z = 1.8

            if self.time_in_state >= self.turn_duration:
                self.state = 'forward'
                self.time_in_state = 0
                self.get_logger().info('Еду вперед...')

        self.pub.publish(msg)

        if self.side_count >= 4:
            self.get_logger().info('=== Квадрат завершен! Начинаю новый ===')
            self.side_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
