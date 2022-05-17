#!/usr/bin/env python

import rospy
import numpy as np
from waypoint_controller import WaypointController


# Each row is a waypoint [x, y, z, yaw]
waypoints = np.zeros((3, 4))
waypoints[0,:] = np.array([0, 0, 3, 0])
waypoints[1,:] = np.array([10, 0, 3, 0])
waypoints[2,:] = np.array([0, 0, 3, np.pi])

if __name__ == '__main__':
    try:
        waypointController = WaypointController(waypoints, 'slung_payload')
        rospy.spin()  
    except rospy.ROSInterruptException:
        print("error!")