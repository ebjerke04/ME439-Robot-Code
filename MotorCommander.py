#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import traceback

from std_msgs.msg import Float32


class MotorCommander(Node):
    
    def __init__(self):
        super().__init__('motor_commander')
        self.publisher_left = self.create_publisher(Float32, 'motor_command_left', 1)
        self.publisher_right = self.create_publisher(Float32, 'motor_command_right', 1)
        
    def operate(self):
        msg_left = Float32()
        msg_right = Float32()
        try:
            while True:
                cmd = input('Enter left and right motor commands (space-separated): ').strip()
                left_speed, right_speed = map(float, cmd.split())
                
                msg_left.data = left_speed
                msg_right.data = right_speed
                
                self.publisher_left.publish(msg_left)
                self.publisher_right.publish(msg_right)
                
                self.get_logger().info('Publishing Left: %+5.3f, Right: %+5.3f' % (msg_left.data, msg_right.data))
        except:
            traceback.print_exc()
        

def main(args=None):
    rclpy.init(args=args)

    motor_commander_instance = MotorCommander()
    motor_commander_instance.operate()
        
    
if __name__ == '__main__':
    main()
