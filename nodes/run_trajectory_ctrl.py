#!/usr/bin/env python

import rospy
import numpy as np
from trajectory_ctrl import *
#from ..utils import pathplanner

rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('ll_controller')

# Each row is a waypoint [x, y, z, yaw]
wp_no = 1
waypoints = np.zeros((wp_no, 3))
waypoints[0,:] = np.array([0, 0, 1])
'''waypoints[1,:] = np.array([5, 0, 2])
waypoints[2,:] = np.array([5, 5, 2])
waypoints[3,:] = np.array([-5, 5, 2])
waypoints[4,:] = np.array([0, 0, 2])
waypoints[5,:] = np.array([0, 0, 0])
waypoints[3,:] = np.array([-5, -5, 2])
waypoints[4,:] = np.array([-5, 5, 2])
waypoints[5,:] = np.array([0, 0, 2])'''

def main():
    try:
        trajectoryController = TrajectoryController(pkg_dir)
        rospy.spin()  
    except rospy.ROSInterruptException:
        print("error!")
    # commented out to not repeat plot
    time_plotter(trajectoryController.drone_states, \
        trajectoryController.setpoints, pkg_dir)

if __name__ == '__main__':
    main()