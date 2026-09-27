Inheritance structure

Sensor
	->Lidar
	->ThermalCamera
	->RgbCamera
 
Robot
	->Parrot
	->Husky

ParrotNode

HuskyNode

SystemControlNode

DataStructures





Tasks for each class
Sensor:
- A base class for all sensors 

Lidar:
- Collects and stores the most recent lidar scan

ThermalCamera:
- Collects and stores the most recent thermal camera image

RgbCamera:
- Collects and stores the most recent camera image


Robot:
- A base clas for all robots

Parot:
- Owns a copy of each sensor
- Has a list of waypoints as targets that can be set by the SystemControlNode
- Image processing (if not in seperate class)
- Position of detected patient (if not in seperate class)


Husky:
- Owns a copy of each sensor
- Has a list of waypoints as targets that can be set by the SystemControlNode



ParrotNode:
- Has a copy of the parot object
- Owns the parrot runloop
- Uses the Nav2
- Decides wether a person has been detected (from multiple confident frames)
- Sends comands to Parrot
- Publishes confirmed casualty coordinate to SystemControlNode
- Publishes Lidar data to SystemControlNode (for the gradient map), or directly to a saperate map class
- Recieves and updated parrot waypoints from SystemControlNode

HuskyNode:
- Has a copy of the Husky object
- Owns the Husky runloop
- Sends comands to Husky
- Publishes Lidar data to SystemControlNode (for the gradient map), or directly to a saperate map class
- Recieves and updated Husky waypoints from SystemControlNode



currently the program recieves and prints out the lidar data recieved from the husky
use command:
ros2 run control_template lidar_test