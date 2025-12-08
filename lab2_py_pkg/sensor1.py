#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class Sensor1(Node):
    def __init__(self):
        super().__init__('sensor1')
        self.ok = True
        self.pub = self.create_publisher(Bool, '/sensor1_state', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.create_service(Trigger, '/sensor1/restart', self.handle_restart)

    def tick(self):
        self.pub.publish(Bool(data=self.ok))
        self.get_logger().info(f'[sensor1]: publishing:{self.ok}')

    def handle_restart(self, req, res):
        self.ok = True
        res.success = True
        res.message = 'sensor1 restarted'
        self.get_logger().warn(res.message)
        return res

def main(args=None):
    rclpy.init(args=args)
    node = Sensor1()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
