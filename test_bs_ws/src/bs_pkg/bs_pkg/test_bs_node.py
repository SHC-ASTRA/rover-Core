import rclpy
from rclpy.node import Node

import pygame

import serial
import sys
import threading
import glob

from std_msgs.msg import String

class BaseStation(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("test_bs")

        self.create_timer(0.25, self.send_controls)


        # Create a publisher to publish any output the pico sends
        self.publisher = self.create_publisher(String, '/astra/core/control', 10) 

        # Create a subscriber to listen to any commands sent for the pico
        self.subscriber = self.create_subscription(String, '/astra/core/feedback', self.read_feedback, 10)
        self.subscriber


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

        # Initialize the first gamepad
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()


    def run(self):
        # This thread makes all the update processes run in the background
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        

        try:
            while rclpy.ok():
                #Check the pico for updates
                self.send_controls()

                self.read_feedback()

        except KeyboardInterrupt:
            sys.exit(0)
        

    def send_controls(self):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        left_x = self.gamepad.get_axis(0)
        left_y = self.gamepad.get_axis(1)
        right_x = self.gamepad.get_axis(2)
        right_y = self.gamepad.get_axis(3)


        output = f'ctrl,{round(left_y,4)},{round(left_y,4)},{round(right_y,4)},{round(right_y,4)}'

        #print(f"[Controls] {output}", end="")
        self.get_logger().info(f"[Ctrl] {output}")
        # Create a string message object
        msg = String()

        # Set message data
        msg.data = output

        # Publish data
        self.publisher.publish(msg)
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