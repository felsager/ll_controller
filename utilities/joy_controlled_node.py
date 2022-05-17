#!/usr/bin/env python

''' Class used for controlling the drone with joystick and setup necessary template for PD controller. '''

import rospy
from math import cos, sin, atan2
import numpy as np
from tf.transformations import quaternion_from_euler
import os
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Joy

def normalize_angle(theta):
  """ Normalize the given angle. """
  return atan2(sin(theta), cos(theta))

os.chdir('/home/felsager/Workspaces/auro_ws/src/ll_controller/csv')

class JoyController:

    def __init__(self):
        """Constructor """
        # Create ROS node 
        rospy.init_node(f'joy_controller')
        self.mode = 0
        print(f'Mode set to attitude control')
        # Joystick subscriber
        self.joystick_sub = rospy.Subscriber('joy', Joy, callback=self.joystick_callback, queue_size=10)
        
        # Publisher for PX4 attitude offboard control
        self.attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10) 

        self.r1 = 0
        self.r2 = 0
        self.r3 = 0
        self.thrust = 0
        
        # Necessary for keeping OFFBOARD on
        self.control_period = 0.1
        self.control_timer = rospy.Timer(rospy.Duration(self.control_period), self.control_callback)

        self.initialize_drone()
        print('done init')

    def joystick_callback(self, data):
        if data.buttons[1]:
            self.change_mode()
        self.r1 = data.axes[3]
        self.r2 = data.axes[4]
        self.r3 = data.axes[0]
        self.thrust = data.axes[1]
        self.thrust_attitude_control(self.r1, self.r2, self.r3, self.thrust)

    def change_mode(self):
        self.mode = not self.mode
        if not self.mode:
            print(f'Mode set to attitude control')
        else:
            print(f'Mode set to body rate control')                  

    def control_callback(self, event):
        self.thrust_attitude_control(self.r1, self.r2, self.r3, self.thrust)

    def thrust_attitude_control(self, r1, r2, r3, thrust_val): 
        """ Manual controller using PX4 built-in with attitude or body rate mode. """
        r1 *= np.pi/2
        r2 *= np.pi/2
        r3 *= np.pi/2
        msg = AttitudeTarget()
        k = 1
        msg.thrust = 0.7 + k*thrust_val # no compensation for body angles implemented for joystick
        if not self.mode:
            msg.type_mask = 3 # ignore roll and pitch body rates
            msg.orientation = Quaternion(*quaternion_from_euler(r1, r2, 0)) # set only roll and pitch angles
            msg.body_rate.z = r3
        else:
            msg.type_mask = 128 # ignore attitude
            msg.body_rate.x = r1
            msg.body_rate.y = r2
            msg.body_rate.z = r3
        self.attitude_pub.publish(msg)

    def initialize_drone(self):
        """ Set mode to "OFFBOARD" and arm the drone. """
        # Publish control setpoints to PX4 controller so mode can be change to "OFFBOARD"
        rate = rospy.Rate(50)
        for i in range(150):
            self.thrust_attitude_control(self.r1, self.r2, self.r3, self.thrust)
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

def test_callback():
    print(f'Test worked :)')

if __name__ == '__main__':
    try:
        JoyController = JoyController()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print("error!")