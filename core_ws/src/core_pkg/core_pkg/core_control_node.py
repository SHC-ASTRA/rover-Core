import rclpy
from rclpy.node import Node

import serial
import sys
import threading
import glob


from std_msgs.msg import String

class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("serial_publisher")#previously 'serial_publisher'

        # Create a publisher to publish any output the pico sends
        self.publisher = self.create_publisher(String, '/astra/core/feedback', 10) 

        # Create a subscriber to listen to any commands sent for the pico
        self.subscriber = self.create_subscription(String, '/astra/core/control', self.send, 10)

        # Loop through all serial devices on the computer to check for the pico
        self.port = None
        ports = SerialRelay.list_serial_ports()
        for port in ports:
            try:
                # connect and send a ping command
                ser = serial.Serial(port, timeout=1)
                ser.write(b"ping\n")
                response = ser.read_until("\n")

                # if pong is in response, then we are talking with the pico
                if b"pong" in response:
                    self.port = port
                    print(f"Found MCU at {self.port}!")
                    break
            except:
                pass
        
        if self.port is None:
            print("Unable to find MCU... please make sure it is connected.")
            sys.exit(1)
        
        self.ser = serial.Serial(self.port, 115200)

    def run(self):
        # This thread makes all the update processes run in the background
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        
        try:
            while rclpy.ok():
                # Check the pico for updates
                self.read_pico()

        except KeyboardInterrupt:
            sys.exit(0)
        

    def read_pico(self):
        output = str(self.ser.readline(), "utf8")
        if output:
            print(f"[Pico] {output}", end="")
            # Create a string message object
            msg = String()

            # Set message data
            msg.data = output

            # Publish data
            self.publisher.publish(msg)
            #print(f"[Pico] Publishing: {msg}")
    
    def send(self, msg):
        command = msg.data + '\n'
        print(f"[Sys] {command}", end="")

        # Send command to pico
        self.ser.write(bytes(command, "utf8"))
        #print(f"[Sys] Relaying: {command}")

    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/tty[A-Za-z]*")
        

def main(args=None):
    rclpy.init(args=args)

    serial_pub = SerialRelay()
    serial_pub.run()


if __name__ == '__main__':
    main()