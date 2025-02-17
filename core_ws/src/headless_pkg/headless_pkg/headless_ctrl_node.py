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
from ros2_interfaces_pkg.msg import CoreControl

max_speed = 75 #Max speed as a duty cycle percentage (1-100)

class Headless(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("headless_ctrl")

        self.create_timer(0.20, self.send_controls)


        # Create a publisher to publish any output the pico sends
        self.publisher = self.create_publisher(CoreControl, '/core/control', 10) 


        self.lastMsg = String() #Used to ignore sending controls repeatedly when they do not change



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

                if pygame.joystick.get_count() == 0: #if controller disconnected, wait for it to be reconnected
                    print(f"Gamepad disconnected: {self.gamepad.get_name()}")
                    
                    while pygame.joystick.get_count() == 0:
                        self.send_controls()
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
        input = CoreControl()

        input.max_speed = max_speed

        input.right_stick = round(self.gamepad.get_axis(4),2)#right y-axis
        if self.gamepad.get_axis(5) > 0:
            input.left_stick = input.right_stick
        else:
            input.left_stick = round(self.gamepad.get_axis(1),2)#lext y-axis



        if pygame.joystick.get_count() != 0:
            output = f'L: {input.left_stick}, R: {input.right_stick}, M: {max_speed}' #stop the rover if there is no controller connected
            self.get_logger().info(f"[Ctrl] {output}")
            self.publisher.publish(input)
        else:
            input.left_stick = 0
            input.right_stick = 0
            input.max_speed = 0
            self.get_logger().info(f"[Ctrl] Stopping. No Gamepad Connected. ")
            self.publisher.publish(input)


        

def main(args=None):
    rclpy.init(args=args)

    node = Headless()
    
    rclpy.spin(node)
    rclpy.shutdown()

    #tb_bs = BaseStation()
    #node.run()


if __name__ == '__main__':
    main()
