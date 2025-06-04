#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pynput import keyboard

class KeyboardJoystick(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.joy_pub_ = self.create_publisher(TwistStamped, '/input_joy/cmd_vel_stamped', 10)
        self.msg = TwistStamped()
        self.key_state = set()
        self.timer = self.create_timer(0.1, self.update_twist)

    def on_press(self, key):
        try:
            self.key_state.add(key.char)
        except AttributeError:
            pass
    
    def on_release(self, key):
        try:
            self.key_state.discard(key.char)
        except AttributeError:
            pass
        if key == keyboard.Key.esc:
            return False

    def update_twist(self):
        self.msg.twist.linear.x = 0.0
        self.msg.twist.angular.z = 0.0

        if 'w' in self.key_state:
            self.msg.twist.linear.x += 0.7
        if 's' in self.key_state:
            self.msg.twist.linear.x -= 0.7
        if 'a' in self.key_state:
            self.msg.twist.angular.z += 0.5
        if 'd' in self.key_state:
            self.msg.twist.angular.z -= 0.5

        self.joy_pub_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJoystick()
    with keyboard.Listener(
            on_press=node.on_press,
            on_release=node.on_release) as listener:
        rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()