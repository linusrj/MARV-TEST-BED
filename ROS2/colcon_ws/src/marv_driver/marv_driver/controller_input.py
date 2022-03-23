import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import pygame
from pygame.locals import *
import sys

from time import sleep

from marv_msgs.msg import CmdSteering
from std_msgs.msg import Int8

class ControllerInput(Node):
    def __init__(self):
        super().__init__('controller_input')
        self.dt = 0.04
        self.killswitch = 0
        self.reset_killswitch = 0

        self.speed = 0.5
        self.setSpeed = 0.5

        self.angle = 0.4
        
        
        self.marv_sys_ctrl_cmdSteering_publisher_ = self.create_publisher(CmdSteering, '/marv/sys/ctrl/cmd_steering', 10)
        
        self.scenario_config_timer = self.create_timer(self.dt, self.update)
    
    def send_cmdSteering(self,aps,rps,angle):
        cmdSteering_message = CmdSteering()
        cmdSteering_message.aps = aps
        cmdSteering_message.rps = rps
        cmdSteering_message.angle = angle
        self.marv_sys_ctrl_cmdSteering_publisher_.publish(cmdSteering_message)

    def update(self):
        for event in pygame.event.get():
            if event.type == JOYBUTTONDOWN:
                if event.button == 0:           # Xbox A button
                    self.killswitch = 1
                if event.button == 4:           # Xbox Y button
                    self.reset_killswitch = 0
            if event.type == JOYBUTTONUP:
                if event.button == 0:           # Xbox A button
                    self.killswitch = 0
                    self.reset_killswitch = 1
            if event.type == JOYAXISMOTION and self.killswitch:
                if event.axis == 4:             # Xbox right trigger, to accelerate
                    self.setSpeed = (round(event.value, 2) + 1) / 4 + 0.5
                if event.axis == 5:             # Xbox left trigger, to brake
                    self.setSpeed = abs(-round(event.value, 2) + 1) / 4
                if event.axis == 0:             # Xbox left joystick, to steer
                    self.angle = abs(-round(event.value, 2) + 1) * 0.4
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()
        
        if self.killswitch and not self.reset_killswitch:
            if self.setSpeed < self.speed and self.speed > 0.01:
                self.speed -= 0.01
            elif self.setSpeed > self.speed:
                self.speed += 0.01
            
            print(f"Speed: {self.speed}, Angle: {self.angle}")
            self.send_cmdSteering(self.speed, 0.0, self.angle)

        if not self.killswitch:
            self.setSpeed = 0.52
            while not(self.speed > 0.51 and self.speed < 0.53):
                if self.speed < 0.53:
                    self.speed += 0.01
                elif self.speed > 0.51:
                    self.speed -= 0.01

                print(f"Speed: {self.speed}, Angle: {self.angle}")
                self.send_cmdSteering(self.speed, 0.0, self.angle)

                sleep(self.dt/2)
        

def main(args=None):
    pygame.init()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joystick in joysticks:
        print(joystick.get_name())


    rclpy.init(args=args)

    input = ControllerInput()

    rclpy.spin(input)

    input.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
