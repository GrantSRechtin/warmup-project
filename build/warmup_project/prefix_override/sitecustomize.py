import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/brenna/ros2_ws/src/warmup_project/install/warmup_project'
