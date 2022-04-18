#!/usr/bin/env python

import quaternion
import rospy
import numpy as np
import os
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import LinkStates

os.chdir('/home/felsager/Workspaces/auro_ws/src/ll_controller/csv')

class WaypointController:

    def __init__(self, waypoints, task):
        """Constructor """
        # Create ROS node 
        rospy.init_node(f'{task}_test')
        self.task = task

        self.control_period = 0.1
        # Timer for control action
        self.control_timer = rospy.Timer(rospy.Duration(self.control_period), self.control_callback)
        # Publisher for PX4 position controller
        self.pose_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10) 
        # Subscriber for getting the position of the drone and payload
        self.state_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_callback) 
        self.time = [0]

        # Robot state
        self.drone_state = []
        self.current_drone_state = [0, 0, 0]
        self.payload_state = []
        self.current_payload_state = [0, 0, 0]

        self.drone_ind = 1
        self.payload_ind = -1

        # Waypoints
        self.waypoints = waypoints
        self.waypoint_no = waypoints.shape[0]
        self.waypoint_ind = 0
        self.current_waypoint = waypoints[0,:]

        self.error_threshold = 0.5

        # Publish control setpoints to PX4 contnroller so mode can be change to "OFFBOARD"
        rate = rospy.Rate(50)
        q = np.quaternion(1, 0, 0, 0)
        for i in range(150):
            self.postion_control(0.0, 0.0, 3.0, q)
            rate.sleep()

        self.initialize_drone()
        print('Going to ', self.current_waypoint)

    def update_waypoint(self):
        """ Increment waypoint index and set current waypoint"""
        self.waypoint_ind += 1
        self.current_waypoint = self.waypoints[self.waypoint_ind]    

    def control_callback(self, event):
        """ Save states, calculate error, set waypoint and send position command. """
        self.drone_state.append(self.current_drone_state[:])
        self.payload_state.append(self.current_payload_state[:])
        self.time.append(self.time[-1] + self.control_period)
        error = self.calculate_error()
        if error < self.error_threshold:
            if self.waypoint_ind == self.waypoint_no - 1:
                np.savetxt(f'{self.task}_time.csv', self.time[1:], delimiter=",")
                np.savetxt(f'{self.task}_drone_state.csv', self.drone_state, delimiter=",")
                np.savetxt(f'{self.task}_payload_state.csv', self.payload_state, delimiter=",")
                rospy.signal_shutdown('Last waypoint has been reached')
                self.time.remove(0)
            self.update_waypoint()
            print('Going to ', self.current_waypoint)
            print(f'{self.waypoint_ind = }')
        x = self.current_waypoint[0]
        y = self.current_waypoint[1]
        z = self.current_waypoint[2]
        q = quaternion.from_euler_angles(0, 0, self.current_waypoint[3])
        print(f'{[x, y, z] = }')
        self.postion_control(x, y, z, q)
                                

    def postion_control(self, x, y, z, q): 
        """ Position controller using PX4 built-in with attitude option. """
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = 1 # q.x
        msg.pose.orientation.y = 0 # q.y
        msg.pose.orientation.z = 0 # q.z
        msg.pose.orientation.w = 0 # q.w # also publish orientation
        self.pose_publisher.publish(msg)
    
    def calculate_error(self):
        """ Calculate error as magnitude of vector from drone to desired waypoint. """
        error_vector = np.subtract(self.current_drone_state, self.current_waypoint[:3]) 
        print(f'{self.current_drone_state}')
        return np.linalg.norm(error_vector)

    def state_callback(self, data):
        """ Update the current state of the drone """
        self.current_drone_state[0] = data.pose[self.drone_ind].position.x
        self.current_drone_state[1] = data.pose[self.drone_ind].position.y
        self.current_drone_state[2] = data.pose[self.drone_ind].position.z
        self.current_payload_state[0] = data.pose[self.payload_ind].position.x
        self.current_payload_state[1] = data.pose[self.payload_ind].position.y
        self.current_payload_state[2] = data.pose[self.payload_ind].position.z


    def initialize_drone(self):
        """ Set mode to "OFFBOARD" and arm the drone. """
        # Set Mode
        print("\nSetting Mode")
        rospy.wait_for_service('/mavros/set_mode')
        try:
            change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode, 5)
            response = change_mode(custom_mode="OFFBOARD")
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" %e)

        # Arm
        print("\nArming")
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool, 5)
            response = arming_cl(value = True)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Arming failed: %s" %e)

