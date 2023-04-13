#!/usr/bin/env python3

import abc

import rclpy
from rclpy import qos
from rclpy.node import Node

from sensor_msgs.msg import Joy
from keyboard_msgs.msg import Key

import yaml


class JoyPart(abc.ABC):
    def __init__(self, init_value):
        self._value = init_value

    @abc.abstractmethod
    def down(self, code):
        pass

    @abc.abstractmethod
    def up(self, code):
        pass

    def get(self):
        return self._value


class Button(JoyPart):
    def __init__(self, key_str):
        super().__init__(0)
        self.code = getattr(Key, key_str)

    def down(self, code):
        if code == self.code:
            self._value = 1

    def up(self, code):
        if code == self.code:
            self._value = 0


class Axis(JoyPart):
    def __init__(self, key_neg_str, key_pos_str):
        super().__init__(0.0)
        self.code_neg = getattr(Key, key_neg_str)
        self.code_pos = getattr(Key, key_pos_str)

    def down(self, code):
        if code == self.code_neg:
            self._value -= 1.0

        elif code == self.code_pos:
            self._value += 1.0

    def up(self, code):
        if code == self.code_neg:
            self._value += 1.0

        elif code == self.code_pos:
            self._value -= 1.0


class KeyboardToJoyNode(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__("keyboard_to_joy_node")

        # Get parameters
        self.declare_parameter("config_file_name")
        config_file_name = (
            self.get_parameter("config_file_name").get_parameter_value().string_value
        )

        self.declare_parameter("sampling_frequency", 50)
        hz = (
            self.get_parameter("sampling_frequency").get_parameter_value().integer_value
        )

        # Load config file
        with open(config_file_name, "rb") as configfile:
            self.config = yaml.load(configfile, Loader=yaml.FullLoader)

        self.buttons = [Button(key_str) for key_str in self.config.get("buttons", [])]
        self.axes = [
            Axis(key_neg_str, key_pos_str)
            for key_neg_str, key_pos_str in self.config.get("axes", [])
        ]

        # Setup publisher
        self.joy = Joy()
        self.joy_pub = self.create_publisher(Joy, "joy", qos.qos_profile_system_default)

        # Keyboard callback
        self.keydown_sub = self.create_subscription(
            Key, "keydown", self.keydown_callback, qos.qos_profile_system_default
        )
        self.keyup_sub = self.create_subscription(
            Key, "keyup", self.keyup_callback, qos.qos_profile_system_default
        )

        # Start timer
        dt = 1.0 / float(hz)
        self.create_timer(dt, self.main_loop)

    def keydown_callback(self, msg):
        for part in self.axes + self.buttons:
            part.down(msg.code)

    def keyup_callback(self, msg):
        for part in self.axes + self.buttons:
            part.up(msg.code)

    def main_loop(self):
        msg = Joy(
            axes=[a.get() for a in self.axes], buttons=[b.get() for b in self.buttons]
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joy_pub.publish(msg)


def main(args=None):
    # Start node, and spin
    rclpy.init(args=args)
    node = KeyboardToJoyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
