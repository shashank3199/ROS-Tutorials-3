#!/usr/bin/env python3

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


class ParameterWriter(Node):
    """A ROS2 node that demonstrates parameter writing in Python."""

    def __init__(self):
        """Initialize the parameter writer node."""
        super().__init__('parameter_writer')

        # Initialize counters and values
        self.update_counter = 0
        self.speed_value = 0.5

        # Declare parameters with descriptors and initial values
        self.declare_parameter(
            'my_robot.name',
            'RoboX',
            ParameterDescriptor(description='Robot name identifier')
        )

        self.declare_parameter(
            'my_robot.debug_mode',
            True,
            ParameterDescriptor(description='Enable/disable debug mode')
        )

        # Declare max_speed with valid range
        self.declare_parameter(
            'my_robot.max_speed',
            self.speed_value,
            ParameterDescriptor(
                description='Maximum robot speed in m/s',
                floating_point_range=[
                    FloatingPointRange(
                        from_value=0.0,
                        to_value=5.0,
                        step=0.1
                    )
                ]
            )
        )

        # Declare sensor list with initial value
        self.declare_parameter(
            'my_robot.sensor_list',
            ['lidar'],
            ParameterDescriptor(description='List of active sensors')
        )

        # Declare sampled parameters with descriptors and initial values
        self.declare_parameter(
            'param_sample_robot.name',
            '',
            ParameterDescriptor(description='Robot name identifier')
        )

        self.declare_parameter(
            'param_sample_robot.debug_mode',
            False,
            ParameterDescriptor(description='Enable/disable debug mode')
        )

        self.declare_parameter(
            'param_sample_robot.max_speed',
            0.0,
            ParameterDescriptor(description='Maximum robot speed in m/s')
        )

        self.declare_parameter(
            'param_sample_robot.sensor_list',
            Parameter(
                'param_sample_robot.sensor_list',
                Parameter.Type.STRING_ARRAY,
                []
            ).get_parameter_value(),
            ParameterDescriptor(description='List of active sensors')
        )

        # Get the sampled parameters
        name = self.get_parameter('param_sample_robot.name').value
        debug_mode = self.get_parameter('param_sample_robot.debug_mode').value
        max_speed = self.get_parameter('param_sample_robot.max_speed').value
        sensor_list = self.get_parameter('param_sample_robot.sensor_list').value

        self.get_logger().info('Sampled parameters:')
        self.get_logger().info(f'Name: {name}')
        self.get_logger().info(f'Debug mode: {debug_mode}')
        self.get_logger().info(f'Max speed: {max_speed}')
        self.get_logger().info(f'Sensor list: {", ".join(sensor_list)}')
        self.get_logger().info('-' * 40)

        # Create timer for periodic parameter updates
        self.timer = self.create_timer(3.0, self.timer_callback)

        self.get_logger().info('Parameter Writer Node initialized')

    def update_single_parameter(self, name, value):
        """
        Update a single parameter and log the result.

        Args:
            name: Parameter name
            value: New parameter value

        Returns:
            bool: True if update was successful
        """
        try:
            param = Parameter(name, value=value)
            result = self.set_parameters([param])[0]

            if result.successful:
                self.get_logger().info(
                    f"Successfully updated parameter '{name}' to: {value}"
                )
                return True
            else:
                self.get_logger().error(
                    f"Failed to update parameter '{name}': {result.reason}"
                )
                return False

        except Exception as e:
            self.get_logger().error(f"Error updating parameter '{name}': {str(e)}")
            return False

    def timer_callback(self):
        """Periodically update parameters with different patterns."""
        try:
            self.update_counter += 1

            # 1. Toggle debug mode
            current_debug = self.get_parameter('my_robot.debug_mode').value
            self.update_single_parameter('my_robot.debug_mode', not current_debug)

            # 2. Increment max_speed
            self.speed_value += 0.5
            if self.speed_value > 5.0:
                self.speed_value = 0.5
            self.update_single_parameter('my_robot.max_speed', self.speed_value)

            # 3. Batch update multiple parameters
            params = []

            # Update robot name with counter
            new_name = f'RoboX_{self.update_counter}'
            params.append(Parameter('my_robot.name', value=new_name))

            # Update sensor list based on counter
            new_sensors = ['lidar']  # Always include lidar
            if self.update_counter % 2 == 0:
                new_sensors.append('camera')
            if self.update_counter % 3 == 0:
                new_sensors.append('ultrasonic')
            params.append(Parameter('my_robot.sensor_list', value=new_sensors))

            # Set all parameters at once
            results = self.set_parameters(params)

            # Check results and log
            all_success = True
            for i, result in enumerate(results):
                if not result.successful:
                    all_success = False
                    self.get_logger().error(
                        f"Failed to update parameter '{params[i].name}': {result.reason}"
                    )

            if all_success:
                self.get_logger().info(f"Robot name updated to: {new_name}")
                self.get_logger().info(f"Sensor list updated to: {', '.join(new_sensors)}")

            # Print separator for log readability
            self.get_logger().info('-' * 40)

        except rclpy.exceptions.ParameterNotDeclaredException as e:
            self.get_logger().error(f"Parameter not declared: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in timer callback: {str(e)}")


def main(args=None):
    """Run the parameter writer node."""
    rclpy.init(args=args)

    try:
        param_writer = ParameterWriter()
        rclpy.spin(param_writer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Unexpected error: {str(e)}')
    finally:
        # Clean up
        if 'param_writer' in locals():
            param_writer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()