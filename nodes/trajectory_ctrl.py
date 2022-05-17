#!/usr/bin/env python

''' Class used for controlling the position of the drone using a velocity tractory. '''

import rospy
import rospkg
import numpy as np
import matplotlib.pyplot as plt
import os
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, TwistStamped, Quaternion, Vector3, PoseStamped
from gazebo_msgs.msg import ModelStates
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float64


def time_plotter(drone_states, setpoint, pkg_dir): # plot errors? plot transitions? do it by making an array which has times appended when waypoint is updated
    os.chdir(f'{pkg_dir}/plots')
    N = len(setpoint)
    t = np.linspace(0, 0.01*N, N)
    axes = 'xyz'
    for i in range(3):
        fig, ax = plt.subplots()
        ax.set_title(f'Drone {axes[i]}-movement vs time')
        ax.plot(t, drone_states[:, i], color='red', label=f'Drone {axes[i]} movement')
        ax.plot(t, setpoint[:, i], ':', color='black', label=f'Drone {axes[i]}-reference')
        ax.set_xlabel(f't [s]')
        ax.set_ylabel(f'{axes[i]} [m]')
        ax.legend(loc='upper left', fancybox=True)
        ax.grid()
        fig.tight_layout()
        fig.savefig(f'drone_{axes[i]}_time_movement_trajectory.svg', format='svg')

class TrajectoryController:

    def __init__(self, pkg_dir):
        """Constructor """
        # Create ROS node 
        rospy.init_node(f'trajectory_ctrl')
        self.pkg_dir = pkg_dir

        # Arrays for saving states of drone
        self.drone_states = []
        self.current_drone_state = [0, 0, 0]
        self.setpoints = []

        # Arrays for saving variables needed in control
        self.position = np.zeros(3)
        self.current_yaw = 0

        # Publisher for PX4 velocity offboard control
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist,
                                             queue_size=1)
        
        self.pose_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Mavros pose and twist subscribers 
        self.velocity_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.velocity_callback, queue_size=10)
        
        # Gazebo ground truth subscriber of drone - replace with vicon for real time
        self.state_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        
        # Change mode to offboard and arm the drone

        self.step = 0
        t1 = np.linspace(0, 5, 501)
        t2 = np.linspace(4.99, 0, 500)
        t = np.append(t1, t2)
        self.v_x = 2*(1-np.abs(t/5-1))
        self.initialize_drone()

        # Control callback necessary for keeping OFFBOARD on
        self.control_period = 0.01
        self.control_timer = rospy.Timer(rospy.Duration(self.control_period), self.control_callback)

        print('Starting trajectory')      

    def velocity_callback(self, data):
        """ Callback for drone velocity subscriber - absolute values are used (magnitude)"""
        self.drone_yaw_rate = data.twist.angular.z

    def update_waypoint(self):
        """ Increment waypoint index and set current waypoint"""
        self.waypoint_ind += 1
        self.int_e = np.zeros(3)
        if self.waypoint_ind < self.waypoint_no:
            self.current_waypoint = self.waypoints[self.waypoint_ind]
            if self.waypoint_types[self.waypoint_ind]:
                self.current_error_threshold = 0.1
            else:
                self.current_error_threshold = 0.3
        waypoint_angle = np.subtract(self.current_waypoint, self.waypoints[self.waypoint_ind-1])
        self.des_yaw = np.arctan2(waypoint_angle[1], waypoint_angle[0])

    def control_callback(self, event):
        """ Callback for trajectory action """
        lin_vel = Vector3(self.v_x[self.step], 0, 0)
        msg = Twist()
        msg.linear = lin_vel
        self.velocity_pub.publish(msg)
        self.step += 1
        if self.step >= 1001:
            print('Trajectory finished')
            rospy.signal_shutdown()

    def state_callback(self, data):
        """ Update the current state of the drone """
        self.current_drone_state[0] = data.pose[1].position.x # index 1 is drone in gazebo
        self.current_drone_state[1] = data.pose[1].position.y
        self.current_drone_state[2] = data.pose[1].position.z
        self.position = self.current_drone_state
        q = data.pose[1].orientation
        self.current_yaw = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))[2]

    def postion_control(self, x, y, z, q): 
        """ Position controller using PX4 built-in with attitude option. """
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = q
        self.pose_publisher.publish(msg)

    def initialize_drone(self):
        """ Set mode to "OFFBOARD" and arm the drone. """
        # Publish control setpoints to PX4 controller so mode can be change to "OFFBOARD"
        rate = rospy.Rate(50)
        q = Quaternion(1, 0, 0, 0)
        for i in range(150):
            self.postion_control(0.0, 0.0, 5.0, q)
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

        # Set drone height
        for i in range(1000000):
            self.postion_control(0.0, 0.0, 5.0, q)
            rate.sleep()