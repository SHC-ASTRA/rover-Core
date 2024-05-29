import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

import signal
import time
import atexit

import serial
import sys
import threading
import glob

from std_msgs.msg import String
from interfaces_pkg.msg import CoreFeedback

serial_pub = None
thread = None

class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("serial_publisher")#previously 'serial_publisher'

        # Create a publisher to publish any output the MCU sends
        self.feedback_publisher = self.create_publisher(String, '/astra/core/feedback', 10) 
        self.telemetry_publisher = self.create_publisher(CoreFeedback, '/astra/core/telemetry', 10)

        # Create a subscriber to listen to any commands sent for the MCU
        self.subscriber = self.create_subscription(String, '/astra/core/control', self.send, 10)
        


        # Create a service server for pinging the rover
        self.ping_service = self.create_service(Empty, '/astra/core/ping', self.ping_callback)

        # Loop through all serial devices on the computer to check for the MCU
        self.port = None
        ports = SerialRelay.list_serial_ports()
        for i in range(2):
            for port in ports:
                try:
                    # connect and send a ping command
                    ser = serial.Serial(port, timeout=1)
                    ser.write(b"ping\n")
                    response = ser.read_until("\n")

                    # if pong is in response, then we are talking with the MCU
                    if b"pong" in response:
                        self.port = port
                        print(f"Found MCU at {self.port}!")
                        break
                except:
                    pass
            if self.port is not None:
                break
        
        if self.port is None:
            print("Unable to find MCU... please make sure it is connected.")
            time.sleep(1)
            sys.exit(1)
        
        self.ser = serial.Serial(self.port, 115200)
        atexit.register(self.cleanup)


        #self.heartbeat_client = self.create_client(Empty, '/astra/core/heartbeat')
        #self.connected: bool = False
        #while not self.heartbeat_client.wait_for_service(timeout_sec=2.0):
        #    self.get_logger().info("Could not connect to Base Station. Retrying...")
    

        #threading.Thread(target=self.heartbeat_thread, daemon=True)
        

    def run(self):
        # This thread makes all the update processes run in the background
        global thread
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        
        try:
            while rclpy.ok():
                # Check the MCU for updates
                self.read_MCU()
                #if not self.connected:
                #    self.send("auto,stop")
        except KeyboardInterrupt:
            sys.exit(0)

    def read_MCU(self):
        try:
            output = str(self.ser.readline(), "utf8")
            
            if output:
                packet = output.strip().split(',')
                
                if len(packet) >= 2 and packet[0] == "core" and packet[1] == "telemetry":
                    feedback = CoreFeedback()
                    feedback.gpslat = float(packet[2])
                    feedback.gpslon = float(packet[3])
                    feedback.gpssat = float(packet[4])
                    feedback.bnogyr.x = float(packet[5])
                    feedback.bnogyr.y = float(packet[6])
                    feedback.bnogyr.z = float(packet[7])
                    feedback.bnoacc.x = float(packet[8])
                    feedback.bnoacc.y = float(packet[9])
                    feedback.bnoacc.z = float(packet[10])
                    feedback.orient = float(packet[11])
                    feedback.bmptemp = float(packet[12])
                    feedback.bmppres = float(packet[13])
                    feedback.bmpalt = float(packet[14])

                    self.telemetry_publisher.publish(feedback)
                else:
                    print(f"[MCU] {output}", end="")
                    msg = String()
                    msg.data = output
                    self.feedback_publisher.publish(msg)
                    return
        except serial.SerialException:
            print("SerialException caught... closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass
        except TypeError as e:
            print(f"TypeError: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass
        except Exception as e:
            print(f"Exception: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass
                


    def send(self, msg):
        command = msg.data + '\n'
        print(f"[Sys] {command}", end="")

        # Send command to MCU
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
    
    def cleanup(self):
        print("Cleaning up...")
        if self.ser.is_open:
            self.ser.close()

def myexcepthook(type, value, tb):
    print("Uncaught exception:", type, value)
    if serial_pub:
        serial_pub.cleanup()
      


def main(args=None):
   rclpy.init(args=args)
   sys.excepthook = myexcepthook

   global serial_pub

   serial_pub = SerialRelay()
   serial_pub.run()

if __name__ == '__main__':
    signal.signal(signal.SIGTSTP, lambda signum, frame: sys.exit(0))  # Catch Ctrl+Z and exit cleanly
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))  # Catch termination signals and exit cleanly
    main()

