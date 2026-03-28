import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from turtle_controller_pkg.srv import SpawnTurtle
from turtlesim.srv import Spawn

class SpawnServer(Node):
    def __init__(self):
        super().__init__('spawn_server')
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            SpawnTurtle, 'spawn_turtle', self.handle_request,
            callback_group=self.cb_group)
        self.spawn_cli = self.create_client(Spawn, '/spawn')

    async def handle_request(self, req, resp):
        
        if not (0 <= req.x <= 11 and 0 <= req.y <= 11):
            resp.success = False
            resp.message = 'Invalid coords, out of range: (0-11)'
            return resp

        spawn_req = Spawn.Request()
        spawn_req.x = float(req.x)
        spawn_req.y = float(req.y)
        spawn_req.name = req.turtle_name
        spawn_req.theta = 0.0
        
        future = self.spawn_cli.call_async(spawn_req)
        result = await future
        
        if result is not None:
            resp.success = True
            resp.message = f'Spawned {req.turtle_name}'
        else:
            resp.success = False
            resp.message = 'Spawn service call failed'
        
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = SpawnServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
