#!/usr/bin/env python

import rospy
import numpy as np
from pd_controller import *

# Each row is a waypoint [x, y, z, yaw]
wp_no = 6
waypoints = np.zeros((wp_no, 3))
waypoints[0,:] = np.array([0, 0, 5])
waypoints[1,:] = np.array([5, 0, 5])
waypoints[2,:] = np.array([-5, 0, 5])
waypoints[3,:] = np.array([0, 5, 5])
waypoints[4,:] = np.array([-5, 0, 5])
waypoints[5,:] = np.array([0, 0, 5])

if __name__ == '__main__':
    try:
        pdController = PDController(waypoints) # testing with mass = 0.5 kg
        rospy.spin()  
    except rospy.ROSInterruptException:
        print("error!")
    # commented out to not repeat plot
    time_plotter(pdController.time, pdController.drone_states, \
    pdController.payload_states, pdController.setpoints)