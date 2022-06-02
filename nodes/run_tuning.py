#!/usr/bin/env python

import rospy
import numpy as np
from pd_controller import *

# Each row is a waypoint [x, y, z, yaw]
wp_no = 2
waypoints = np.zeros((wp_no, 3))
waypoints[0,:] = np.array([0, 0, 1])
waypoints[1,:] = np.array([10, 0, 1])

if __name__ == '__main__':
    try:
        pdController = PDController(waypoints) # testing with mass = 0.5 kg
        rospy.spin()  
    except rospy.ROSInterruptException:
        print("error!")
    # commented out to not repeat plot
    time_plotter(pdController.time, pdController.drone_states, \
    pdController.payload_states, pdController.setpoints)