# dpm15-w19
Source Code written in Java for ECSE-211 (Design Principles and Methods) Final Robotics Competition, Team 15, Winter 2019 Session. 
It works on LEGO Mindstorms EV3 platform under LeJOS firmware. 


## Project Overview
Design a robot capable of navigating autonomously on the field, in search for 210 mL cans of various color and weight combinations. A can is to be retrieved and placed upright at the robot's starting position. 
- Robot must successfully bring back a can within 5 minutes. 
- Robot cannot travel into the water
..* Robot must travel through teh tunnel


## Components used
1. Color Sensor x 3
2. Ultrasonic Sensor x 1
3. Large EV3 Motor x 2
4. Large NXT Motor x 1
5. Medium Motor x 1


## Software Flow
1. Initialize all sensors
2. Receive info through Wi-Fi
3. Start odometer
4. Start localization
5. Generate the list of sequence of points to travel to
6. Travel through the tunnel to the target search zone
7. Search for a can
8. Once it finds a can, detect the color and weight (Red/Green/Blue/Yellow & Heavy/Light)
9. Grab a can
10. Go back to the starting position
11. Drop a can

## Testing Results
- Localization: 88.89%
- Navigation: 68.75%
- Searching: 75.00%
- Color Classification & Weighing: 85.7%
