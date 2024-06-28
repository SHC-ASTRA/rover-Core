import rclpy
from rclpy.node import Node

import pygame

import time

import serial
import sys
import threading
import glob

from std_msgs.msg import String

class BaseStation(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("test_bs")

        self.create_timer(0.10, self.send_controls)


        # Create a publisher to publish any output the pico sends
        self.publisher = self.create_publisher(String, '/astra/core/control', 10) 

        # Create a subscriber to listen to any commands sent for the pico
        self.subscriber = self.create_subscription(String, '/astra/core/feedback', self.read_feedback, 10)
        self.subscriber


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


        left_x = self.gamepad.get_axis(0)#left x-axis
        left_y = self.gamepad.get_axis(1)#lext y-axis
        left_t = self.gamepad.get_axis(2)#left trigger
        right_x = self.gamepad.get_axis(3)#right x-axis
        right_y = self.gamepad.get_axis(4)#right y-axis
        right_t = self.gamepad.get_axis(5)#right trigger



        if right_t > 0:#single-stick control mode
            output = f'ctrl,{round(right_y,4)},{round(right_y,4)}'
        else:
            output = f'ctrl,{round(left_y,4)},{round(right_y,4)}'


        #print(f"[Controls] {output}", end="")
        self.get_logger().info(f"[Ctrl] {output}")
        # Create a string message object
        msg = String()

        # Set message data
        msg.data = output

        #only publish commands when the values are updated
        if self.lastMsg != msg:
            # Publish data
            self.publisher.publish(msg)
            
            self.lastMsg = msg
            #print(f"[Pico] Publishing: {msg}")


    def read_feedback(self, msg):
        
        # Create a string message object
        #msg = String()

        # Set message data
        #msg.data = output

        # Publish data
        #self.publisher.publish(msg.data)
        
        print(f"[Pico] {msg.data}", end="")
        #print(f"[Pico] Publishing: {msg}")

        

def main(args=None):
    rclpy.init(args=args)

    node = BaseStation()
    
    rclpy.spin(node)
    rclpy.shutdown()

    #tb_bs = BaseStation()
    #node.run()


if __name__ == '__main__':
    main()
