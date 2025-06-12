import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ayesha/Documents/OpenProj/aerial-drones/src/drone_racing_ros2/tello_ros/tello_comm/install/tello_comm'
