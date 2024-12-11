import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/endr/Developer/Robotics/Reference/ROS2/ros2_ws/src/install/pkg2'
