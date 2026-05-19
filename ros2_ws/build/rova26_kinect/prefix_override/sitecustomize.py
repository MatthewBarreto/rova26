import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mattthewhumble/rova26/ros2_ws/install/rova26_kinect'
