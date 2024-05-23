import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

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
                   print(f"Found pico at {self.port}!")
                   break
           except:
               pass
      
       if self.port is None:
           print("Unable to find pico... please make sure it is connected.")
           sys.exit(1)
      
       self.ser = serial.Serial(self.port, 115200)

       self.heartbeat_client = self.create_client(Empty, '/astra/core/heartbeat')
       self.connected: bool = False
       while not self.heartbeat_client.wait_for_service(timeout_sec=2.0):
           self.get_logger().info("Could not connect to Base Station. Retrying...")


       threading.Thread(target=self.heartbeat_thread, daemon=True)

   def run(self):
       # This thread makes all the update processes run in the background
       thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
       thread.start()
      
       try:
           while rclpy.ok():
               # Check the pico for updates
               self.read_pico()
               if not self.connected:
                   self.send("auto,stop")


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


   def ping_callback(self, request, response):
       return response

   def heartbeat_thread(self):
       while True:
           response = self.heartbeat_client.call_async(Empty.Request())
           rclpy.spin_until_future_complete(self, response, timeout_sec=3.0)
           if response:
               self.connected = True
           else:
               self.connected = False


   @staticmethod
   def list_serial_ports():
       return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
      


def main(args=None):
   rclpy.init(args=args)

   serial_pub = SerialRelay()
   serial_pub.run()

if __name__ == '__main__':
   main()

