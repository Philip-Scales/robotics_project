#!/bin/bash

#hacky script to save time...

echo "Starting nodes..."
cd ~/catkin_ws
source devel/setup.bash
gnome-terminal -e "bash -c 'rosrun follow_me moving_persons_detector_node;/bin/bash'"
gnome-terminal -e "bash -c 'rosrun follow_me robot_moving_node;/bin/bash'"
gnome-terminal -e "bash -c 'rosrun follow_me translation_action_node;/bin/bash'"
gnome-terminal -e "bash -c 'rosrun follow_me obstacle_detection_node;/bin/bash'"
gnome-terminal -e "bash -c 'rosrun follow_me decision_node;/bin/bash'"
gnome-terminal -e "bash -c 'rosrun follow_me '"
gnome-terminal -e "bash -c 'echo hi there; sleep 2;/bin/bash'"
echo "Robot is running. Type \"k\" to kill all terminals"

input=""

while [ "$input" != "k" ]
do
    read input
done

killall bash
bash
#yes, I kill the current terminal. Feel free to fix it if you have better ideas
