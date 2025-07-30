#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch offboard base.launch;exec bash"' \
--tab -e 'bash -c "sleep 2;roslaunch offboard mission.launch;exec bash"' \
--tab -e 'bash -c "sleep 4;source ~/anaconda3/bin/activate yolov8;cd ~/robocup/src/robocup_yolo/;conda activate yolov8;python3 target_detection.py;exec bash"' \

