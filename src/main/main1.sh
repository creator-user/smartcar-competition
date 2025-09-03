gnome-terminal -- bash -c "\
roslaunch main start.launch; \
exec bash"


gnome-terminal -- bash -c "\
cd /home/ucar/Desktop/ucar/src/main/navacation;\
python3 navacation-tracking.py;\

exec bash"
