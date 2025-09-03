# CMake generated Testfile for 
# Source directory: /home/ucar/Desktop/ucar/src/YDLidar-SDK/python
# Build directory: /home/ucar/Desktop/ucar/src/YDLidar-SDK/build/python
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ydlidar_py_test "/home/ucar/archiconda3/bin/python" "/home/ucar/Desktop/ucar/src/YDLidar-SDK/python/test/pytest.py")
set_tests_properties(ydlidar_py_test PROPERTIES  ENVIRONMENT "PYTHONPATH=/home/ucar/Desktop/ucar/devel/lib/python3/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages:\$PYTHONPATH:/home/inano/.local/lib/python3.6/site-packages:/home/ucar/Desktop/ucar/src/YDLidar-SDK/build/python")
subdirs("examples")
