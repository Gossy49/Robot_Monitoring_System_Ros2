#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ReportError
from std_srvs.srv import Trigger


class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler')

        # get parameter from YAML (default 2.0)
        self.declare_parameter('restart_wait', 2.0)
        self.wait_time = self.get_parameter('restart_wait').value

        # service to receive errors
        self.create_service(ReportError, '/report_error', self.handle_error)
        self.get_logger().info(f'ErrorHandler ready. Restart wait: {self.wait_time}s')

    def handle_error(self, request, response):
        sensor = request.sensor_name
        self.get_logger().warn(f'Error from {sensor}')

        srv_name = f'/{sensor}/restart'
        client = self.create_client(Trigger, srv_name)

        if client.wait_for_service(timeout_sec=self.wait_time):
            self.get_logger().info(f'Calling {srv_name}')
            client.call_async(Trigger.Request())
            response.accepted = True
            response.info = f'Restart called for {sensor}'
        else:
            self.get_logger().error(f'{srv_name} not found')
            response.accepted = False
            response.info = f'{srv_name} not found'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ErrorHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
