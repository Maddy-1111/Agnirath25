# Team Agnirath
We are a team designing and building a solar powered race car to race in World Solar Challenge.\
The competition is an endurance race in Australia where we begin at Darwin and end at Adelaide (3000km)

## Solar Model
This has 2 main goals:
1. Accurately estimate the total solar incident power and energy on our car throughout the race in various conditions.
2. Design the optimal array position and orientation to maximize the total energy recieved.

To do this, We:
1. Interface with the Mech team to balance aerodynamic loss while maximising incident energy
2. Interface with the Strat team to determine energy flows and thus the forcaset position and velocity of the car through the race


## Telemetry System
The telemetry system is responsible for reliably collecting the sensor data of the car and relaying it to a chase vehicle roughly 1km behind  
It contains:
1. The LoRa code:
    - These are to be uploaded onto an ESP32 (any arduino style microcontroller works too)
    - The Transmitter is on the car connected to a E32-900T30D module (seperate branch contains the AI-433 module codes) and to the onboard computer
    - An identical hardware setup is present on the Receiver side, with the Reciever codes instead

2. The ROS codes:
    - This consists of the Uplink node on the onboard computer which compacts the data, formats it to packets as mentioned in the packet_structure.json file
    - The packets sent have the following structure: [header] [crc] [length] [type] [data]

3. The Downlink code:
    - This consists of a python file which unpacks the data and sends it out as a dictionary of sensor values
    - Code here logs it, but in the competition, its integrated with a dashboard to view live data
  

## Jetson Codes:
The Jetson is the onboard computer and this contains the ROS2 codes that are required to:
1. Read and process CAN Data
2. Send Camera feed to the onboard Display
3. Collect additional sensor data such as the GPS feed and IMU feed