# core-Rover
This is the repository for Astra's Core Rover code. 

## Software Pre-reqs


The BaseStation computer will need several things.

- ROS2 Humble

- Follow the standard ROS2 humble install process. Linux recommended.

- https://docs.ros.org/en/humble/Installation.html

- Colcon

- `$ sudo apt update`

- `$ sudo apt install python3-colcon-common-extensions`

- Configured Static IP for Ubiquiti bullet

- This process will depend on the system, but you'll need this for using the Ubiquiti network with the M2 bullets.

- Settings:

- Net Mask: 255.255.255.0

- IP Address: 192.168.1.x

- This can be just about anything, just not ending in 1.20, 1.21, 1.69, or 1.0

- Gateway: 192.168.1.0
  

## Headless Control

  To control the rover without a base station you'll need to run the headless control node. 
  
- SSH to the the rover from a computer computer over wifi (or connect directly)

- `ssh clucky@192.168.1.69`

- Password: spaceiscool639

- Run `Ros2 run headless_pkg headless`
	- This needs to be done in a separate ssh/shell window from the core rover node

  

## Running the Rover node
  

- Ensure you're either connected directly to the rover's network switch over ethernet or connected to an M2 bullet which has connection to the rover (red signal lights on)

- SSH to the the rover from a computer computer over wifi (or connect directly)

- `ssh clucky@192.168.1.69`

- Password: spaceiscool639

- run `ros2 run core_control_pkg core_control`
	- This needs to be done in a separate ssh/shell window from the headless control node (if you're running it)

- You should see "MCU found" on a serial line

- If not, you'll get an error it's not found and the program will close. Ensure the teensy is connected via usb to the Nuc

  

## Connecting the GuliKit King Kong Pro 2 Controller (Recommended)

  

- Connect controller to pc with USB-C

- Select the "X-Input" control mode (Windows logo) on the controller.

- Hold the button next to the symbols (windows, android, switch, etc...)

- You'll need to release the button and press down again to cycle to the next mode


