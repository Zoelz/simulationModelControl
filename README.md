# simulationModelControl

Program for running smart factory under centralized client-server, decentralized client-server or decentralized peer-to-peer control.

Example videos can be found from folder "videos".

# Running program

Each program are run its onw terminal and ROS setup.bash should be run on each terminal.

1. Program is run by first starting the Gazebo environment. (gazebo --verbose GazeboEnvironment.world)
2. Then starting OPCUA servers.
3. After that the ros2opcua programs
4. Control servers that are used (AI version when running decentralized peer-to-peer implementation)
5. running roll.py (decentralized client-server) or client.py (centralized client.py)

# Requirements

pip install -r requirements.py

ROS Foxy

Gazebo
