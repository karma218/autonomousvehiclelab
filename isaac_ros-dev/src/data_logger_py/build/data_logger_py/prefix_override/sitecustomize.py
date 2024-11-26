import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/snow/autonomousvehiclelab/isaac_ros-dev/src/data_logger_py/install/data_logger_py'
