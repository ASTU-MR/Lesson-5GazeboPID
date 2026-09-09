#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult


class PID(Node):
    config = {}

    sp = 0.0
    e_sum = 0.0
    e_old = 0.0

    def __init__(self):
        super().__init__(f'pid')
        self.config['kp'] = self.declare_parameter('kp', 0.0).value
        self.config['ki'] = self.declare_parameter('ki', 0.0).value
        self.config['kd'] = self.declare_parameter('kd', 0.0).value
        self.config['clamp'] = self.declare_parameter('clamp', 1.0).value
        self.config['angular'] = self.declare_parameter('angular', False).value
        self.config['inverted'] = self.declare_parameter('inverted', False).value
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().warn(f"Parameters updated {self.config}")

        self.setpoint_subscription = self.create_subscription(Float64, '~/setpoint',
                                                              self.sp_callback, 10)

        self.state_subscription = self.create_subscription(Float64, '~/state',
                                                           self.state_callback, 10)

        self.pub_result = self.create_publisher(Float64, '~/output', 10)

    def parameter_callback(self, data):
        for parameter in data:
            self.config[parameter.name] = parameter.value

        self.get_logger().warn(f"Parameters updated {self.config}")
        return SetParametersResult(successful=True)

    def sp_callback(self, msg):
        if self.sp != msg.data:
            self.sp = msg.data
            self.get_logger().warn(f"PID setpoint changed to {self.sp}")
            self.e_sum = 0.0
            self.e_old = 0.0

    def state_callback(self, msg):
        message = Float64()
        message.data = self.update(msg.data)
        # self.get_logger().warn(f"PID sp: {self.sp}; state: {msg.data}: effort: {message.data}")
        self.pub_result.publish(message)

    def angular_constraint(self, e):
        return (e + 180.0) % (2 * 180.0) - 180.0

    def update(self, state):
        error = self.sp - state if not self.config['inverted'] else state - self.sp
        if self.config['angular']:
            error = self.angular_constraint(error)
        self.get_logger().warn(f"PID error: {error}")
        p = self.config['kp'] * error
        self.e_sum += self.config['ki'] * error
        self.e_sum = min(self.config['clamp'], max(self.e_sum, -self.config['clamp']))
        d = self.config['kd'] * (error - self.e_old)
        self.e_old = error
        PID = p + self.e_sum + d
        return min(self.config['clamp'], max(PID, -self.config['clamp']))


def main():
    rclpy.init()

    signal_corrector = PID()

    rclpy.spin(signal_corrector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    signal_corrector.destroy_node()
    rclpy.shutdown()
