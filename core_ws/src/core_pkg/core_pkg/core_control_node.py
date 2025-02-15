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
from ros2_interfaces_pkg.msg import CoreFeedback
from ros2_interfaces_pkg.msg import CoreControl

serial_pub = None
thread = None

class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("serial_publisher")#previously 'serial_publisher'

        # Create publishers for feedback and telemetry
        self.debug_pub = self.create_publisher(String, '/core/debug', 10) 
        self.feedback_pub = self.create_publisher(CoreFeedback, '/core/feedback', 10)

        # Create a subscriber to listen to any commands sent for the MCU
        self.control_sub = self.create_subscription(CoreControl, '/core/control', self.send_controls, 10)
        

        # Create a service server for pinging the rover
        self.ping_service = self.create_service(Empty, '/astra/core/ping', self.ping_callback)

        # Loop through all serial devices on the computer to check for the MCU
        self.port = None
        ports = SerialRelay.list_serial_ports()
        for i in range(2):
            for port in ports:
                try:
                    # connect and send a ping command
                    ser = serial.Serial(port, 115200, timeout=1)
                    print(f"Checking port {port}...")
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
            print("Unable to find MCU...")
            time.sleep(1)
            sys.exit(1)
        
        self.ser = serial.Serial(self.port, 115200)
        atexit.register(self.cleanup)


    def run(self):
        # This thread makes all the update processes run in the background
        global thread
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        
        try:
            while rclpy.ok():
                self.read_MCU() # Check the MCU for updates
        except KeyboardInterrupt:
            sys.exit(0)

    def read_MCU(self):
        try:
            output = str(self.ser.readline(), "utf8")
            
            if output:
                # All output over debug temporarily
                print(f"[MCU] {output}", end="")
                msg = String()
                msg.data = output
                self.debug_pub.publish(msg)
                return
                # Temporary

                # packet = output.strip().split(',')
                
                # if len(packet) >= 2 and packet[0] == "core" and packet[1] == "telemetry":
                #     feedback = CoreFeedback()
                #     feedback.gpslat = float(packet[2])
                #     feedback.gpslon = float(packet[3])
                #     feedback.gpssat = float(packet[4])
                #     feedback.bnogyr.x = float(packet[5])
                #     feedback.bnogyr.y = float(packet[6])
                #     feedback.bnogyr.z = float(packet[7])
                #     feedback.bnoacc.x = float(packet[8])
                #     feedback.bnoacc.y = float(packet[9])
                #     feedback.bnoacc.z = float(packet[10])
                #     feedback.orient = float(packet[11])
                #     feedback.bmptemp = float(packet[12])
                #     feedback.bmppres = float(packet[13])
                #     feedback.bmpalt = float(packet[14])

                #     self.telemetry_publisher.publish(feedback)
                # else:
                #     # print(f"[MCU] {output}", end="")
                #     # msg = String()
                #     # msg.data = output
                #     # self.debug_pub.publish(msg)
                #     return
        except serial.SerialException as e:
            print(f"SerialException: {e}")
            print("Closing serial port.")
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
                

    def send_controls(self, msg):
        #can_relay_tovic,core,19, left_stick, right_stick 
        command = "can_relay_tovic,core,19," + msg.left_stick/(msg.max_speed/100.0) + ',' + msg.right_stick/(msg.max_speed/100.0) + '\n'
        #print(f"[Sys] {command}", end="")
        
        self.ser.write(bytes(command, "utf8"))# Send command to MCU
        self.get_logger().debug(f"wrote: {command}")
        #print(f"[Sys] Relaying: {command}")


    def ping_callback(self, request, response):
        return response


    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    
    def cleanup(self):
        print("Cleaning up before terminating...")
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

