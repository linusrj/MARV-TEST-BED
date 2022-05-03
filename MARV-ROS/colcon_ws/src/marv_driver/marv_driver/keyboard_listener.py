import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Messages
from std_msgs.msg import UInt64
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Float32

from marv_msgs.msg import CmdSteering
from marv_msgs.msg import Keyboard


class Listener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')

        self.speed = 0.5
        self.angle = 0.4

        #self.marv_sys_ctrl_cmdSteering_publisher_ = self.create_publisher(CmdSteering, '/marv/sys/ctrl/cmd_steering', 10)

        #self.subscription = self.create_subscription(Keyboard, 'keyboard/binary', self.callback, 10)

        #self.scenario_config_timer = self.create_timer(1, self.stop)
    
    def send_cmdSteering(self,aps,rps,angle):
        cmdSteering_message = CmdSteering()
        cmdSteering_message.aps = aps
        cmdSteering_message.rps = rps
        cmdSteering_message.angle = angle
        self.marv_sys_ctrl_cmdSteering_publisher_.publish(cmdSteering_message)

    def callback(self, msg):
        #print(f"Up: {msg.k_up}, Down: {msg.k_down}, Left: {msg.k_left}, Right: {msg.k_right}\n")

        if (msg.k_up and not msg.k_down and self.speed < 1.0) or (not msg.k_down and not msg.k_up and self.speed <= 0.54):
            self.speed += 0.01

        if (msg.k_down and not msg.k_up and self.speed > 0.01) or (not msg.k_down and not msg.k_up and self.speed > 0.54):
            self.speed -= 0.01

        if (msg.k_right and not msg.k_left and self.angle > 0.01) or (not msg.k_left and not msg.k_right and self.angle >= 0.4):
            self.angle -= 0.05

        if (msg.k_left and not msg.k_right and self.angle < 0.8) or (not msg.k_left and not msg.k_right and self.angle < 0.4):
            self.angle += 0.05
        
        self.send_cmdSteering(self.speed, 0.0, self.angle)

        print(f"Speed: {self.speed}, Angle: {self.angle}")

def main(args=None):
    rclpy.init(args=args)

    li = Listener()

    rclpy.spin(li)

    li.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
