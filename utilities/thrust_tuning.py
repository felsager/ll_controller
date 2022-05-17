#!/usr/bin/env python

''' Class used for controlling the position of the drone using a PD controller. '''

import quaternion
import rospy
from math import cos, sin, atan2
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import os
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import AttitudeTarget

def normalize_angle(theta):
  """ Normalize the given angle. """
  return atan2(sin(theta), cos(theta))

os.chdir('/home/felsager/Workspaces/auro_ws/src/ll_controller/csv')

class ThrustTuning:

    def __init__(self, waypoints):
        """Constructor """
        # Create ROS node 
        rospy.init_node(f'thrust_tuning')
        
        # Publisher for PX4 attitude offboard control
        self.attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10) 
        
        # Necessary for keeping OFFBOARD on
        self.control_period = 0.01
        self.control_timer = rospy.Timer(rospy.Duration(self.control_period), self.control_callback)

        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback, queue_size=10) 
        self.velocity_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.velocity_callback, queue_size=10)
        
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)

        # PD coefficients
        self.kp_thrust = 1.5
        self.kd_thrust = -0.5

        self.waypoints = np.array(waypoints)
        self.waypoint_no = waypoints.shape[0]
        self.waypoint_ind = 0
        self.current_waypoint = waypoints[0,:]

        self.error_threshold = 0.2

        self.initialize_drone()
        print('done init')           

    def pose_callback(self, data):
        """ Callback for drone pose subscriber """
        self.position[0] = data.pose.position.x
        self.position[1] = data.pose.position.y
        self.position[2] = data.pose.position.z
        q = data.pose.orientation
        self.orientation = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))

    def velocity_callback(self, data):
        """ Callback for drone velocity subscriber"""
        self.velocity[0] = data.twist.linear.x
        self.velocity[1] = data.twist.linear.y
        self.velocity[2] = data.twist.linear.z

    def control_callback(self, event):
        """ Callback for control action """
        error = self.get_position_error()
        thrust = self.thrust_controller(error[2], \
            self.velocity[2], self.orientation[0], self.orientation[0])
        #rates = self.rate_controller([])
        msg = AttitudeTarget()
        msg.type_mask = 7
        msg.thrust = thrust
        msg.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        self.attitude_pub.publish(msg)

    def get_position_error(self):
        """ Calculate position error as vector from drone to desired waypoint. """
        error = np.subtract(self.current_waypoint, self.position) 
        return error

    def thrust_controller(self, e_z, v_z, pitch, roll):
        angle_compensation = abs(np.cos(pitch)*np.cos(roll))
        if angle_compensation < 0.25: # compensate angles higher than pi/3 (saturation)
            angle_compensation = 0.25    
        thrust = 0.7 + self.kp_thrust*e_z + self.kd_thrust*v_z
        thrust /= angle_compensation
        return thrust

    def rate_controller(self,):
        pass

    def initialize_drone(self):
        """ Set mode to "OFFBOARD" and arm the drone. """
        # Publish control setpoints to PX4 controller so mode can be change to "OFFBOARD"
        rate = rospy.Rate(50)
        for i in range(150):
            msg = AttitudeTarget()
            msg.type_mask = 4
            msg.thrust = 2
            msg.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
            self.attitude_pub.publish(msg)
            rate.sleep()
        
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
