import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import sys

import json

from time import sleep

from std_msgs.msg import String
from marv_msgs.msg import CmdSteering
from std_msgs.msg import Int8

class WebInput(Node):
    def __init__(self):
        super().__init__('web_input')

        self.gamepad = {}

        self.dt = 0.03
        self.killswitch = 0
        self.reset_killswitch = 0

        self.speed = 0.5
        self.setSpeed = 0.5

        self.angle = 0.4

        self.feedback = String()
        
	# Subscriptions
        self.subscription = self.create_subscription(String, '/web_ui/gamepad', self.receive_input, 10)
	
	# Publishers
        self.cmdSteering_publisher = self.create_publisher(CmdSteering, '/marv/sys/ctrl/cmd_steering', 10)
        self.feedback_publisher = self.create_publisher(String, '/web_ui/feedback', 10)
        
        self.scenario_config_timer = self.create_timer(self.dt, self.send_input)
    
    def send_cmdSteering(self,aps,rps,angle):
        cmdSteering_message = CmdSteering()
        cmdSteering_message.aps = aps
        cmdSteering_message.rps = rps
        cmdSteering_message.angle = angle
        self.cmdSteering_publisher.publish(cmdSteering_message)

    def receive_input(self, msg):
        try:
            self.gamepad = json.loads(msg.data)
        except:
            None

    def send_input(self):
        #print(self.gamepad)
        if self.gamepad.get("button0"):
            self.killswitch = 1
        if self.gamepad.get("button3"):
            self.reset_killswitch = 0
        if not self.gamepad.get("button0"):
            self.killswitch = 0
            self.reset_killswitch = 1

        if self.killswitch and not self.reset_killswitch:
            self.angle = abs(-round(float(self.gamepad.get("lJoy")), 2) + 1) * 0.4

            lTrigVal = abs(-round(float(self.gamepad.get("lTrig")), 2)) / 2
            rTrigVal = (round(float(self.gamepad.get("rTrig")), 2)) / 2 + 0.5

            if lTrigVal >= rTrigVal:
                self.setSpeed = lTrigVal - rTrigVal
            else:
                self.setSpeed = rTrigVal - lTrigVal
            
            if self.setSpeed < self.speed and self.speed > 0.01:
                if self.speed >= 0.52:
                    self.speed -= 0.02
                else:
                    self.speed -= 0.01
            elif self.setSpeed > self.speed:
                if self.speed < 0.52:
                    self.speed += 0.02
                else:
                    self.speed += 0.01          
            

        if not self.killswitch:
            self.setSpeed = 0.52
            if not(self.speed > 0.5 and self.speed < 0.56):
                if self.speed < 0.54:
                    self.speed += 0.04
                elif self.speed > 0.50:
                    self.speed -= 0.04

        
        self.send_cmdSteering(self.speed, 0.0, self.angle)


        ### Send feedback to the web operator
        self.feedback.data = f"APS: {self.speed}, RPS: 0, Angle: {self.angle}"
        self.feedback_publisher.publish(self.feedback)
        

def main(args=None):
    rclpy.init(args=args)

    input = WebInput()

    rclpy.spin(input)

    input.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
