#!/usr/bin/env python3
''' 
# --------------------------------------- #
# MARV Remote Network Steering Scenario   #
# By Linus Johansson and                  #
# Joakim Osterman, Spring 2022            #
# Chalmers University of Technology       #
# --------------------------------------- #
'''

# System imports
import time
import numpy as np
import os

import json

# Custom libraries
from timer import Timer
from scenario import Scenario
from thrust_controller import ThrustController
from heading_controller import HeadingController

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

# Messages
from std_msgs.msg import Bool
from std_msgs.msg import String
from marv_msgs.msg import CmdSteering


class Remote_Network_Steering(Scenario):

    ##################################### INIT #############################################################

    def __init__(self):
        '''The Init function is run on node start. Should not subscribe, publish or start any timers
            except for the "run_setup_check_timer". This is because we don't want anything running in the 
            background when this scenario is not running.
        '''
        # Scenario name parameter (this needs to be added/same as the name in /marv_driver/config/scenarios.yaml)
        __scenario_name = "RNS"
        # Scenario data variable headers, set to "" if not used, max 7 characters
        __data_var_1_header = "SPEED"
        __data_var_2_header = "ANGLE"

        # Initialize Scenario Wrapper (handles the logic for starting and stopping scenarios)
        super().__init__('remote_network_steering',__scenario_name,__data_var_1_header,__data_var_2_header)

        self.get_logger().info("Initializing Scenario Node: " + __scenario_name)

        # Declare rosparams
        self.declare_parameters(
            namespace='',
            parameters=[
                ('thrust_controller_gain', None),
                ('thrust_anti_windup_gain', None),
                ('aps_signal_limit_min', None),
                ('aps_signal_limit_max', None),
                ('surge_rate_limit', None),
                ('heading_controller_gain', None),
                ('heading_anti_windup_gain', None),
                ('steering_angle_limit', None),
                ('steering_angle_rate_limit', None),
                ('u_min', None),
                ('delta_t', None),
                ('vel_diff_ang', None),
                ('ref_pos', None),
                ('wp_file_name', None),
                ('update_freq', None),
                ('wait_time', None),
                ('start_sound_time', None),
            ])

        # ------- Global variables -------
        self.run_setup = True # Whenever to run the setup method when in INITIALIZING mode

        # Timers
        self.node_start_delay_timer = Timer()
        self.u_vel_update_timeout = 1 # second before counting the velocity input data as too outdated, signaling fault
        
        # State variables
        #self.u_vel = 0.0 # Current velocity
        #self.r_vel = 0.0 # Current angular velocity
        #self.eta = np.array([0, 0, 0]) # Current position state: x,y,psi

        # Controller signal
        self.aps_signal = 0.0
        self.steering_angle = 0.0
        # --------------------------------

        # ------- Constants --------------
        # Thrust, APS and surge constants
        self.thrust_controller_gain = self.get_parameter('thrust_controller_gain').value
        self.thrust_anti_windup_gain = self.get_parameter('thrust_anti_windup_gain').value
        self.aps_signal_limit_min = self.get_parameter('aps_signal_limit_min').value
        self.aps_signal_limit_max = self.get_parameter('aps_signal_limit_max').value
        self.surge_rate_limit = self.get_parameter('surge_rate_limit').value

        # Heading and steering constants
        self.heading_controller_gain = self.get_parameter('heading_controller_gain').value
        self.heading_anti_windup_gain = self.get_parameter('heading_anti_windup_gain').value
        self.steering_angle_limit = self.get_parameter('steering_angle_limit').value
        self.steering_rate_limit = self.get_parameter('steering_angle_rate_limit').value

        # Other parameters
        self.u_min = self.get_parameter('u_min').value
        self.delta_t = self.get_parameter('delta_t').value
        self.vel_diff_ang = self.get_parameter('vel_diff_ang').value
        self.ref_pos = self.ref_pos = self.get_parameter('ref_pos').value
        self.wp_file_name = self.get_parameter('wp_file_name').value
        self.update_freq = self.get_parameter('update_freq').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_sound_time = self.get_parameter('start_sound_time').value
        # --------------------------------

        # Configuration parameters 
        self.main_loop_update_period = 1/self.update_freq # seconds update time for the main loop, where the controller should be
        self.get_new_params_period = 5.0 # seconds update time for updating with new rosparams
        
        # Load heading refshape fir coefficients
        hrf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/heading_fir.csv'
        self.heading_ref_fir_coefficients = np.loadtxt(hrf_file_path, delimiter = ',')

        # Load heading yaw fir coefficients
        hyf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/yaw_fir.csv'
        self.heading_yaw_fir_coefficients = np.loadtxt(hyf_file_path, delimiter = ',')

        # Load thrust fir coefficients
        tf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/thrust_fir.csv'
        self.thrust_fir_coefficients = np.loadtxt(tf_file_path, delimiter = ',')

        # Other variables
        self.start_sound_sent = False
        self.start_sound_signal_id = 4
        self.controllers_created = False # Run controller setup once per run


        ### RNS Init stuff ###
        self.gamepad = {}

        self.killswitch = 0
        self.reset_killswitch = 0 
        
        self.speed = 0.0   #self.aps_signal = 0.0
        self.angle = 0.0      #self.steering_angle = 0.0

        self.setSpeed = 0.0

        self.dt = 0.03

    


        # Check setup timer config
        self.__run_setup_check_timer_period = 0.1 # seconds between checking if the setup should be run (which should be done when the scenario is running)

        # Check setup timer
        self.__run_setup_check_timer = self.create_timer(self.__run_setup_check_timer_period,self.__run_setup_check_callback)

        self.get_logger().info("Started Scenario Node: " + __scenario_name)

    ########################################################################################################

    ##################################### SETUP, TEARDOWN AND CHECK ########################################

    def __run_setup_check_callback(self):
        # If scenario is being initialized, run setup
        if self.run_setup and self.get_scenario_state() == "INITIALIZING":
            self.run_setup = False # Prevent the setup from being run several times
            self.__setup() # Call setup
        elif not self.run_setup and self.get_scenario_state() == "STOPPED":
            self.__teardown()
            self.run_setup = True

    def __setup(self):
        '''Setup configuration parameters, global variables, subscriptions, publishes and timers.
            This function is run when the scenario is initializing, i.e. requested to start but not 
            yet runnung (we are still in manual mode which needs to be changed to external to start)
        '''

        # Subscriptions (subscribe to for example sensor data)
        self.gamepad_input = self.create_subscription(String, 'web_ui/gamepad/msg', self.receive_input, 10)

        # Publishes
        #self.marv_sys_ctrl_cmdSteering_publisher = self.create_publisher(CmdSteering, '/marv/sys/ctrl/cmd_steering', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        # Custom timers
        self.node_start_delay_timer.stop()

        # Other variables
        self.start_sound_sent = False
        self.controllers_created = False

        # Timers
        # Set a timer for the main_loop_callback (where the scenario is running)
        self.main_loop_timer = self.create_timer(self.main_loop_update_period, self.main_loop_callback)
        #self.get_new_params_timer = self.create_timer(self.get_new_params_period, self.update_params_callback)

    def __teardown(self):
        '''Teardown timers, publishers and subscriptions when the scenario is no longer running'''
        
        self.destroy_subscription(self.gamepad_input)
        self.destroy_subscription(self.marv_sys_ctrl_cmdSteering_publisher)

        self.main_loop_timer.cancel()
        #self.get_new_params_timer.cancel()

    ########################################################################################################

    ##################################### MAIN LOOP CALLBACK ###############################################

    # Main program loop
    def main_loop_callback(self):
        '''When the external mode is activated, get_scenario_running() will turn true. After that
            the steering commands needs to be updated with at least 10 Hz, using self.update_cmd_steering(), otherwise there will be
            a system error since the PDU will sense that the updates has stopped.
        '''

        if self.get_scenario_running():
            # Start timer to wait before starting
            if self.node_start_delay_timer.elapsed() == 0:
                self.node_start_delay_timer.start()
                self.aps_signal = 0.0
                self.steering_angle = 0.0
                self.get_logger().info("Timer started")

            # Wait
            elif self.node_start_delay_timer.elapsed() > 0 and self.node_start_delay_timer.elapsed() <= self.wait_time:
                if self.wait_time - self.node_start_delay_timer.elapsed() > 0:
                    self.set_progress(str(round(self.wait_time - self.node_start_delay_timer.elapsed(),2))) # Show start countdown
                else:
                    self.set_progress(str(0.00))

                self.set_data_var_1(str("--"))
                self.set_data_var_2(str("--"))

                # Send the buzzer start signal
                if self.start_sound_sent == False and (self.wait_time - self.node_start_delay_timer.elapsed() < self.start_sound_time) and self.start_sound_time != 0:
                    self.start_sound_sent = True
                    self.set_buzzer_signal(self.start_sound_signal_id)
                    self.update_buzzer_signal()
                    self.get_logger().info("Buzzer signal sent")

            # Start
            elif self.node_start_delay_timer.elapsed() > self.wait_time:
                
                
                if self.gamepad.get("button0"):
                    self.killswitch = 1
                if self.gamepad.get("button3"):
                    self.reset_killswitch = 0
                if not self.gamepad.get("button0"):
                    self.killswitch = 0
                    self.reset_killswitch = 1

                if self.killswitch and not self.reset_killswitch:
                    self.steering_angle = -self.gamepad.get("lJoy") * 22     # Jetski positive angle = left steer

                    #lTrigVal = abs(-round(float(self.gamepad.get("lTrig")), 2)) / 2
                    #rTrigVal = (round(float(self.gamepad.get("rTrig")), 2)) / 2 + 0.5

                    #if lTrigVal >= rTrigVal:
                    #    self.setSpeed = lTrigVal - rTrigVal
                    #else:
                    #    self.setSpeed = rTrigVal - lTrigVal
                    
                    #if self.setSpeed < self.speed and self.speed > 0.01:
                    #    if self.speed >= 0.52:
                    #        self.speed -= 0.02
                    #    else:
                    #        self.speed -= 0.01
                    #elif self.setSpeed > self.speed:
                    #    if self.speed < 0.52:
                    #        self.speed += 0.02
                    #    else:
                    #        self.speed += 0.01

                    #print(f"Speed: {self.speed}, Angle: {self.angle}")
                    #self.send_cmdSteering(self.speed, 0.0, self.angle)            
                    

                #if not self.killswitch:
                #    self.setSpeed = 0.52
                #    while not(self.speed > 0.5 and self.speed < 0.56):
                #        if self.speed < 0.54:
                #            self.speed += 0.04
                #        elif self.speed > 0.50:
                #            self.speed -= 0.04

                #        print(f"Speed: {self.speed}, Angle: {self.angle}")
                #        self.send_cmdSteering(self.speed, 0.0, self.angle)

                #        sleep(self.dt)






                # Get new control signals
                self.aps_signal = 0.0
                

                # Set steering commands
                self.set_cmd_steering_aps(self.aps_signal)
                self.set_cmd_steering_angle(self.steering_angle)

            self.update_cmd_steering() # Send the complete steering command to the system
            self.update_all_variables() # Send the updated variables to the system

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    def receive_input(self, msg):
        self.gamepad = json.loads(msg.data)

    # Get current velocity from INS
    def marv_velocity_callback(self,ros_msg):
        self.u_vel = ros_msg.x # Surge

    def marv_angular_velocity_callback(self,ros_msg):
        self.r_vel = ros_msg.z # Yaw

    # Get current pose from INS
    def marv_pose_callback(self,ros_msg):
        self.eta[0] = ros_msg.pose.position.x
        self.eta[1] = ros_msg.pose.position.y
        eta_pose_quat = self.quaternion_to_euler_deg(ros_msg.pose.orientation.x, ros_msg.pose.orientation.y, ros_msg.pose.orientation.z, ros_msg.pose.orientation.w)
        self.eta[2] = eta_pose_quat[2]

    # Set reference position
    def marv_ref_pos_state_callback(self,ros_msg):
        if ros_msg.data == False:
            ref_pos_message = Vector3()
            ref_pos_message.x = float(self.ref_pos[0])
            ref_pos_message.y = float(self.ref_pos[1])
            ref_pos_message.z = float(self.ref_pos[2])
            self.ref_pos_publisher_.publish(ref_pos_message)

    def update_params_callback(self):
        # Thrust, APS and surge constants
        self.thrust_controller_gain = self.get_parameter('thrust_controller_gain').value
        self.thrust_anti_windup_gain = self.get_parameter('thrust_anti_windup_gain').value
        self.aps_signal_limit_min = self.get_parameter('aps_signal_limit_min').value
        self.aps_signal_limit_max = self.get_parameter('aps_signal_limit_max').value
        self.surge_rate_limit = self.get_parameter('surge_rate_limit').value

        # Heading and steering constants
        self.heading_controller_gain = self.get_parameter('heading_controller_gain').value
        self.heading_anti_windup_gain = self.get_parameter('heading_anti_windup_gain').value
        self.steering_angle_limit = self.get_parameter('steering_angle_limit').value
        self.steering_rate_limit = self.get_parameter('steering_angle_rate_limit').value

        # Other parameters
        self.u_min = self.get_parameter('u_min').value
        self.delta_t = self.get_parameter('delta_t').value
        self.vel_diff_ang = self.get_parameter('vel_diff_ang').value
        self.ref_pos = self.get_parameter('ref_pos').value
        self.wp_file_name = self.get_parameter('wp_file_name').value
        self.update_freq = self.get_parameter('update_freq').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_sound_time = self.get_parameter('start_sound_time').value

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################




    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    remote_network_steering = Remote_Network_Steering()

    rclpy.spin(remote_network_steering)

    remote_network_steering.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
