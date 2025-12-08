#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from custom_interfaces.srv import ReportError


class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')

        # get check period from YAML 
        self.declare_parameter('check_period', 1.0)
        period = self.get_parameter('check_period').value

        # set up publisher, subscriber, service
        self.sensor1_state = None
        self.sensor2_state = None
        self.sub1 = self.create_subscription(Bool, '/sensor1_state', self.sensor1_callback, 10)
        self.sub2 = self.create_subscription(Bool, '/sensor2_state', self.sensor2_callback, 10)
        self.pub = self.create_publisher(Bool, '/robot_state', 10)
        self.client = self.create_client(ReportError, '/report_error')

        # timer uses parameter
        self.timer = self.create_timer(period, self.check_system)

        self.get_logger().info(f'StateMonitor started, checking every {period} seconds')

    def sensor1_callback(self, msg):
        self.sensor1_state = msg.data

    def sensor2_callback(self, msg):
        self.sensor2_state = msg.data

    def check_system(self):
        if self.sensor1_state is None or self.sensor2_state is None:
            self.get_logger().info('Waiting for both sensors...')
            return

        ok = self.sensor1_state and self.sensor2_state
        msg = Bool()
        msg.data = ok
        self.pub.publish(msg)

        if ok:
            self.get_logger().info('Robot OK')
        else:
            self.get_logger().warn('Robot ERROR detected')
            if not self.sensor1_state:
                self.report_error('sensor1')
            if not self.sensor2_state:
                self.report_error('sensor2')

    def report_error(self, sensor_name):
        if not self.client.service_is_ready():
            self.get_logger().warn('ErrorHandler not ready')
            return
        
        req = ReportError.Request()
        req.sensor_name = sensor_name
        self.client.call_async(req)
        self.get_logger().info(f'Reporting error from {sensor_name}')


def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
