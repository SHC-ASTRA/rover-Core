import rclpy
from rclpy.node import Node

import pygame

import time

import serial
import sys
import threading
import glob

import importlib
from std_msgs.msg import String
from ...ROS2_Interfaces.interfaces_pkg.msg import ControllerState

class Headless(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("headless_ctrl")

        self.create_timer(0.20, self.send_controls)


        # Create a publisher to publish any output the pico sends
        self.publisher = self.create_publisher(ControllerState, '/astra/core/control', 10) 

        # Create a subscriber to listen to any commands sent for the pico
        self.subscriber = self.create_subscription(String, '/astra/core/feedback', self.read_feedback, 10)
        #self.subscriber


        self.lastMsg = String() #Used to ignore sending controls repeatedly when they do not change


        # Initialize pygame
        pygame.init()

        # Initialize the gamepad module
        pygame.joystick.init()

        # Check if any gamepad is connected
        if pygame.joystick.get_count() == 0:
            print("No gamepad found.")
            pygame.quit()
            exit()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        # Initialize the first gamepad, print name to terminal
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        print(f'Gamepad Found: {self.gamepad.get_name()}')
        #
        #


    def run(self):
        # This thread makes all the update processes run in the background
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        

        try:
            while rclpy.ok():
                #Check the pico for updates
                self.send_controls()

                self.read_feedback()
                if pygame.joystick.get_count() == 0: #if controller disconnected, wait for it to be reconnected
                    print(f"Gamepad disconnected: {self.gamepad.get_name()}")
                    
                    while pygame.joystick.get_count() == 0:
                        self.send_controls()
                        self.read_feedback()
                    self.gamepad = pygame.joystick.Joystick(0)
                    self.gamepad.init() #re-initialized gamepad
                    print(f"Gamepad reconnected: {self.gamepad.get_name()}")
                    

        except KeyboardInterrupt:
            sys.exit(0)
        

    def send_controls(self):
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        input = ControllerState()

        left_x = self.gamepad.get_axis(0)#left x-axis
        left_y = self.gamepad.get_axis(1)#lext y-axis
        left_t = self.gamepad.get_axis(2)#left trigger
        right_x = self.gamepad.get_axis(3)#right x-axis
        right_y = self.gamepad.get_axis(4)#right y-axis
        right_t = self.gamepad.get_axis(5)#right trigger


        if pygame.joystick.get_count() != 0:
        
            if right_t > 0:#single-stick control mode
                output = f'{round(right_y,2)},{round(right_y,2)}'
                input.left_y = round(right_y,2)
                input.right_y = round(right_y,2)
            else:
                output = f'{round(left_y,2)},{round(right_y,2)}'
                input.left_y = round(left_y,2)
                input.right_y = round(right_y,2)
        else:
            input.left_y = 0
            input.right_y = 0
            output = 'ctrl,0,0' #stop the rover if there is no controller connected


        #print(f"[Controls] {output}", end="")
        self.get_logger().info(f"[Ctrl] {output}")
      


        self.publisher.publish(input)
        #removed the check to remove duplicate messages back-to-back
        #if inpput != self.lastMsg: then don't send the message


    def read_feedback(self, msg):
        
        # Create a string message object
        #msg = String()

        # Set message data
        #msg.data = output

        # Publish data
        #self.publisher.publish(msg.data)
        
        print(f"[MCU] {msg.data}", end="")
        #print(f"[Pico] Publishing: {msg}")

        

def main(args=None):
    rclpy.init(args=args)

    node = Headless()
    
    rclpy.spin(node)
    rclpy.shutdown()

    #tb_bs = BaseStation()
    #node.run()


if __name__ == '__main__':
    main()
