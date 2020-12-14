#!/bin/bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

source ~/catkin_ws/devel/setup.bash

echo "SAN OPIL AUTORUN"

# Run SAN container
screen -dmS san_docker bash -c 'cd ~/catkin_ws/src/opil_san_sensors/docker; docker-compose up;  exec bash'   

# Run SAN-ROS bridge
screen -dmS opil_san_sensors bash -c 'sleep 15; cd ~/catkin_ws/; source devel/setup.bash; roslaunch opil_san_sensors multiple_client.launch; exec bash' 

