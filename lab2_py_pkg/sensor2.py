#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class Sensor2(Node):
    def __init__(self):
        super().__init__('sensor2')
        self.ok = True
        self.start_time = time.time()

        # publisher
        self.pub = self.create_publisher(Bool, '/sensor2_state', 10)
        self.timer = self.create_timer(0.5, self.publish_state)

        # restart service
        self.restart_srv = self.create_service(Trigger, '/sensor2/restart', self.restart_sensor)

    def publish_state(self):
        # fail after 5 seconds
        if time.time() - self.start_time > 5.0:
            self.ok = False
        msg = Bool()
        msg.data = self.ok
        self.pub.publish(msg)
        self.get_logger().info(f'sensor2 publishing: {self.ok}')

    def restart_sensor(self, request, response):
        self.ok = True
        self.start_time = time.time()
        response.success = True
        response.message = 'sensor2 restarted'
        self.get_logger().warn('sensor2 restarted')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Sensor2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
