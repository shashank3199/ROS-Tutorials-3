#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_event_handler import ParameterEventHandler
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue, ParameterType

class ParameterModifier(Node):
    def __init__(self):
        super().__init__('parameter_modifier')

        # Create parameter event subscription
        self.param_subscriber_ = ParameterEventHandler(self)

        # Create service client for the parameter reader node
        self.remote_node_name = "parameter_reader"
        self.param_client_ = self.create_client(
            SetParameters,
            f"/{self.remote_node_name}/set_parameters"
        )

        # Create timer to periodically toggle reader status
        self.toggle_timer_ = self.create_timer(5.0, self.toggle_reader_status)

        # Initialize active state
        self.is_active = False

        self.get_logger().info('Parameter Modifier Node initialized')

    def toggle_reader_status(self):
        if not self.param_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Parameter service not available")
            return

        request = SetParameters.Request()

        # Toggle between "active" and "inactive"
        self.is_active = not self.is_active

        # Create parameter message
        status_param = ParameterMsg()
        status_param.name = "/reader_status"
        status_param.value.type = ParameterType.PARAMETER_STRING
        status_param.value.string_value = "active" if self.is_active else "inactive"
        request.parameters = [status_param]

        # Send the request asynchronously
        future = self.param_client_.call_async(request)
        future.add_done_callback(self._toggle_callback)

    def _toggle_callback(self, future):
        try:
            response = future.result()
            if response.results[0].successful:
                self.get_logger().info(
                    f"Successfully toggled reader status to: {'active' if self.is_active else 'inactive'}")
            else:
                self.get_logger().error(
                    f"Failed to toggle reader status: {response.results[0].reason}")
        except Exception as e:
            self.get_logger().error(f"Error in toggle callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ParameterModifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()