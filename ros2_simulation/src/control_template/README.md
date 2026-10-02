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




Parot:
- Has a list of waypoints as targets that can be set by the SystemControlNode
- Image processing (if not in seperate class)
- Position of detected patient (if not in seperate class)


Husky:
- Has a list of waypoints as targets that can be set by the SystemControlNode



ParrotNode:

- Owns the parrot runloop
- Uses the Nav2
- Decides wether a person has been detected (from multiple confident frames)
- Sends comands to Parrot
- Publishes confirmed casualty coordinate to SystemControlNode
- Publishes Lidar data to SystemControlNode (for the gradient map), or directly to a saperate map class
- Recieves and updated parrot waypoints from SystemControlNode

HuskyNode:
- Owns the Husky runloop
- Sends comands to Husky
- Publishes Lidar data to SystemControlNode (for the gradient map), or directly to a saperate map class
- Recieves and updated Husky waypoints from SystemControlNode


