import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/docker_user/ros2_ws_head/install/sensors_bringup'
