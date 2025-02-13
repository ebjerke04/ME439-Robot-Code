#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import traceback

from std_msgs.msg import Float32
from gpiozero import PhaseEnableMotor


class MotorExecutorLeft(Node):

    def __init__(self):
        super().__init__('motor_executor_left')
        self.motor_left = PhaseEnableMotor(5,12)
        self.subscription = self.create_subscription(Float32, 'motor_command_left', self.set_motor_command_left, 1)

    def set_motor_command_left(self, msg_in): 
        motor_command_left = float(msg_in.data)
        self.get_logger().info('Received: %+5.3f' % msg_in.data)
        if motor_command_left >= 0:
            self.motor_left.forward(motor_command_left)
        else:
            self.motor_left.backward(-1*motor_command_left)
            
    def stop_motor_left(self): 
        self.motor_left.stop()

def main(args=None):
    rclpy.init(args=args)

    motor_executor = MotorExecutorLeft()

    try:
        # "spin" will block the program until an error occurs, e.g. keyboard break Ctrl+C. 
        rclpy.spin(motor_executor)
    except: 
        # When an error occurs, catch it with "except" and stop the motors
        motor_executor.stop_motor_left()
        motor_executor.get_logger().info('Stopping Left Motor') 

    rclpy.shutdown()


# Section to start the execution.  
if __name__ == "__main__":
    main()