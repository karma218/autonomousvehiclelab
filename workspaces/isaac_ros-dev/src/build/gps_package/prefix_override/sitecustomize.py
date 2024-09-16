import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/agxorin1/autonomousvehiclelab/workspaces/isaac_ros-dev/src/install/gps_package'
