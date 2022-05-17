import subprocess
import rospy

px4_ekf2_path = '/home/felsager/Documents/PX4-Autopilot/build/px4_sitl_default/bin/px4-ekf2'

ekf2_stop = subprocess.Popen([px4_ekf2_path, 'stop'])
ekf2_start = subprocess.Popen([px4_ekf2_path, 'start'])