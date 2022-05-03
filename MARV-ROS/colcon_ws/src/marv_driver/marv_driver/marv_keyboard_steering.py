#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV Status Sender                    #
# By Anders Logren and                  #
# Linus Johansson, Spring 2022          #
# Chalmers University of Technology     #
# ------------------------------------- #
'''

# System imports
from matplotlib.pyplot import new_figure_manager
import numpy as np
import time
import datetime
import serial

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Messages
from std_msgs.msg import UInt64
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from marv_msgs.msg import CmdSteering

class MARV_Status_Sender(Node):

    ##################################### INIT #############################################################
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_keyboard_steering')

        self.speed = 0.5
        self.angle = 0.5

        self.speedMultiplier = 1
        self.angleMultiplier = 1

        #Publish to cmdsteering
        self.marv_sys_ctrl_cmdSteering_publisher_ = self.create_publisher(CmdSteering, '/marv/sys/ctrl/cmd_steering', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)


        #self.scenario_config_timer = self.create_timer(0.1, self.send_cmd_callback)

    ########################################################################################################


    def send_cmdSteering(self,aps,rps,angle):
        cmdSteering_message = CmdSteering()
        cmdSteering_message.aps = aps
        cmdSteering_message.rps = rps
        cmdSteering_message.angle = angle
        self.marv_sys_ctrl_cmdSteering_publisher_.publish(cmdSteering_message)

    
    def send_cmd_callback(self):
        # Set cmdSteering output to 0
        if self.speed > 0.8: 
            self.speedMultiplier = -1
        if self.speed < 0.2:
            self.speedMultiplier = 1

        if self.angle > 0.8:
            self.angleMultiplier = -1
        if self.angle < 0.2:
            self.angleMultiplier = 1

        self.speed += 0.01 * self.speedMultiplier
        self.angle += 0.01 * self.angleMultiplier

        self.send_cmdSteering(self.speed, 0.0, self.angle)


def main(args=None):
    rclpy.init(args=args)

    marv_status_sender = MARV_Status_Sender()

    rclpy.spin(marv_status_sender)

    marv_status_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
