import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ibuki/ros2_diff_drive/ros2_diff_drive_sim/imu_pid_controller/install/imu_pid_controller'
