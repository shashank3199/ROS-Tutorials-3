#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.parameter import parameter_value_to_python

class ParameterReader(Node):
    def __init__(self):
        super().__init__('parameter_reader')

        # Declare parameters with initial values
        self.declare_parameter("/reader_status", "inactive")

        # Create parameter event subscription
        self.param_subscriber_ = ParameterEventHandler(self)

        # Callback function that handles parameter changes
        def parameters_callback(parameter):
            current_reader_status = self.get_parameter("/reader_status").value == "active"
            if not current_reader_status:
                self.get_logger().warn(
                    "Reader is inactive. Cannot update parameters.")
                return

            self.get_logger().info(
                f"Parameter '{parameter.name}' changed to: {parameter_value_to_python(parameter.value)}")

        # Subscribe to parameters
        self.remote_node_name: str = "/parameter_writer"
        self.remote_param_names = [
            "name",
            "debug_mode",
            "max_speed",
            "sensor_list"
        ]

        # Create callback handles list
        self.cb_handles_ = []

        # Subscribe to parameters
        for param_name in self.remote_param_names:
            full_param_name = f"my_robot.{param_name}"
            handle = self.param_subscriber_.add_parameter_callback(
                parameter_name=full_param_name,
                node_name=self.remote_node_name,
                callback=parameters_callback
            )
            self.cb_handles_.append(handle)

        self.get_logger().info('Parameter Reader Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()