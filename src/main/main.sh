#!/bin/bash
gnome-terminal -- bash -c "\
roslaunch main start.launch; \
exec bash"

gnome-terminal -- bash -c "\
cd /home/ucar/Desktop/ucar/src/main;\
python3 obstacle_avoidance.py;\
exec bash"

gnome-terminal -- bash -c "\
cd /home/ucar/Desktop/ucar/src/main;\
python3 t_dis.py;\
exec bash"

gnome-terminal -- bash -c "\
cd /home/ucar/Desktop/ucar/src/main/navacation;\
python3 navacation-super.py;\
exec bash"