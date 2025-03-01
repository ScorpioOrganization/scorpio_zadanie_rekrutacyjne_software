#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pynput import keyboard

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_front_left = self.create_publisher(Float32, '/vesc/wheel_lf/set_target_velocity', 10)
        self.publisher_front_right = self.create_publisher(Float32, '/vesc/wheel_lr/set_target_velocity', 10)
        self.publisher_rear_left = self.create_publisher(Float32, '/vesc/wheel_rf/set_target_velocity', 10)
        self.publisher_rear_right = self.create_publisher(Float32, '/vesc/wheel_rr/set_target_velocity', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_velocity = 0.0
        self.current_steering = 0.0
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.keys_pressed = set()

    def on_press(self, key):
        try:
            self.keys_pressed.add(key.char)
        except AttributeError:
            self.keys_pressed.add(key)

    def on_release(self, key):
        try:
            self.keys_pressed.remove(key.char)
        except KeyError:
            self.keys_pressed.remove(key)
        except AttributeError:
            self.keys_pressed.remove(key)

    def timer_callback(self):
        if 'w' in self.keys_pressed:
            self.current_velocity = 100.0
        if 's' in self.keys_pressed:
            self.current_velocity = 100.0
        if 'a' in self.keys_pressed:
            self.current_steering = 100.0
        if 'd' in self.keys_pressed:
            self.current_steering = 100.0
        if keyboard.Key.space in self.keys_pressed:
            self.current_velocity = 0.0
            self.current_steering = 0.0
        if 'p' in self.keys_pressed:
            rclpy.shutdown()
            return

        # Calculate velocities for each wheel based on current velocity and steering
        front_left_velocity = Float32()
        front_right_velocity = Float32()
        rear_left_velocity = Float32()
        rear_right_velocity = Float32()

        # Example value assignment; adjust them according to your vehicle's kinematics
        front_left_velocity.data = self.current_velocity + self.current_steering
        front_right_velocity.data = self.current_velocity - self.current_steering
        rear_left_velocity.data = self.current_velocity + self.current_steering
        rear_right_velocity.data = self.current_velocity - self.current_steering

        self.publisher_front_left.publish(front_left_velocity)
        self.publisher_front_right.publish(front_right_velocity)
        self.publisher_rear_left.publish(rear_left_velocity)
        self.publisher_rear_right.publish(rear_right_velocity)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()