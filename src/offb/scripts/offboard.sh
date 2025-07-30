#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch offboard offboard.launch;exec bash"' \
--tab -e 'bash -c "sleep 2;cd /home/nuc/robocup/src/offboard_pkg/scripts/;python3 drop.py;exec bash"' \
