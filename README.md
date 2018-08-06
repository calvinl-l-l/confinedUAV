# Project Swiftlet

UAV for tunnel inspection. Newer version of quadcopter platform based on the QAV500 branch. This code runs alongside with custom Arducopter firmware. 

## Program structure:
### Odroid
- read lidar data
- send lidar data as "fake" sensor data to Pixhawk 2
- control arduino to GPIO (LED signalling)

### Arducopter 3.6-dev (5/8/2018 ver)
- receive fake sensor data from Odroid
- added new "tunnel" flight mode 
	- run an integral backstepping position controller
	- auto takeoff (future TODO)
	- constant forward speed/angle for "auto" mission in tunnel (future TODO)

### Arduino
- runs ChibiOS
- LED signalling 
- servo control
