#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy


class JoySpeedControl(Node):
    def __init__(self):
        super().__init__('joy_speed_control')
        self.declare_parameter(
            'speed_levels',
            [0.50, 0.65, 0.80, 0.95, 1.12])
        self.declare_parameter('initial_level', 3)
        self.declare_parameter('normal_speed', 0.40)
        self.declare_parameter('speed_axis', 4)  # F710 horizontal D-pad
        self.declare_parameter('speed_up_direction', -1.0)  # right
        self.declare_parameter('turbo_button', 1)       # B
        self.declare_parameter('preset_level', 3)
        self.declare_parameter('preset_axis', 7)        # D-pad vertical
        self.declare_parameter('preset_axis_direction', -1.0)  # D-pad down
        self.declare_parameter('preset_axis_threshold', 0.5)

        self.levels = [float(x)
                       for x in self.get_parameter('speed_levels').value]
        if (len(self.levels) != 5 or
                any(not math.isfinite(x) or x <= 0 or x > 1.12 for x in self.levels) or
                any(a >= b for a, b in zip(self.levels, self.levels[1:]))):
            raise ValueError(
                'speed_levels requires 5 increasing speeds in (0, 1.12]')
        self.level = max(
            1, min(5, int(self.get_parameter('initial_level').value)))
        self.last_buttons = []
        self.last_axes = []
        self.dirty = True
        self.client = AsyncParameterClient(self, '/teleop_node')
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(0.10, self.sync_parameters)

        self.log_speed('Joystick speed ready')

    def normal_speed(self):
        return float(self.get_parameter('normal_speed').value)

    def turbo_speed(self):
        return self.levels[self.level - 1]

    def log_speed(self, prefix):
        self.get_logger().info(
            f'{prefix}: level={self.level}/5, '
            f'A(normal)={self.normal_speed():.2f} m/s, '
            f'B(turbo)={self.turbo_speed():.2f} m/s, B+D-pad left=-, right=+, down=3/5')

    def rising(self, buttons, index):
        now = 0 <= index < len(buttons) and bool(buttons[index])
        before = 0 <= index < len(self.last_buttons) and bool(
            self.last_buttons[index])
        return now and not before

    @staticmethod
    def axis_active(axes, index, direction, threshold):
        if not 0 <= index < len(axes):
            return False
        return float(axes[index]) * float(direction) >= float(threshold)

    def rising_axis(self, axes, index, direction, threshold):
        now = self.axis_active(axes, index, direction, threshold)
        before = self.axis_active(self.last_axes, index, direction, threshold)
        return now and not before

    def on_joy(self, msg):
        turbo = int(self.get_parameter('turbo_button').value)
        axis = int(self.get_parameter('speed_axis').value)
        up = float(self.get_parameter('speed_up_direction').value)
        threshold = float(self.get_parameter('preset_axis_threshold').value)
        preset_axis = int(self.get_parameter('preset_axis').value)
        preset_direction = float(
            self.get_parameter('preset_axis_direction').value)
        old_level = self.level
        held = 0 <= turbo < len(msg.buttons) and bool(msg.buttons[turbo])
        if held:
            # Preset wins on a diagonal. A held D-pad never repeats at Joy rate.
            if self.rising_axis(msg.axes, preset_axis, preset_direction, threshold):
                self.level = max(
                    1, min(5, int(self.get_parameter('preset_level').value)))
            elif self.rising_axis(msg.axes, axis, up, threshold):
                self.level = min(5, self.level + 1)
            elif self.rising_axis(msg.axes, axis, -up, threshold):
                self.level = max(1, self.level - 1)
        self.last_buttons = list(msg.buttons)
        self.last_axes = list(msg.axes)
        if self.level != old_level:
            self.dirty = True
            self.log_speed('Speed changed')

    def sync_parameters(self):
        if not self.dirty or not self.client.services_are_ready():
            return
        self.client.set_parameters([
            Parameter('scale_linear.x', value=self.normal_speed()),
            Parameter('scale_linear_turbo.x', value=self.turbo_speed()),
        ])
        self.dirty = False


def main(args=None):
    rclpy.init(args=args)
    node = JoySpeedControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
