#!/usr/bin/env python

''' Class used for controlling the position of the drone using a PD controller. '''

import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import os
from geometry_msgs.msg import TwistStamped, Quaternion
from gazebo_msgs.msg import LinkStates
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import AttitudeTarget
import matplotlib.pyplot as plt

def time_plotter(t, drone_states, payload_states, setpoint):
    os.chdir('/home/felsager/Workspaces/auro_ws/src/ll_controller/plots')
    axes = 'xyz'
    for i in range(3):
        fig, ax = plt.subplots()
        ax.set_title(f'Slung payload {axes[i]}-movement vs time')
        ax.plot(t, drone_states[:, i], color='red', label=f'Drone {axes[i]} movement')
        ax.plot(t, payload_states[:, i], '--', color='blue', label=f'Payload {axes[i]} movement')
        ax.plot(t, setpoint[:, i], ':', color='black', label=f'Drone {axes[i]}-reference')
        ax.set_xlabel(f't [s]')
        ax.set_ylabel(f'{axes[i]} [m]')
        ax.legend(loc='upper left', fancybox=True)
        ax.grid()
        fig.tight_layout()
        fig.savefig(f'slung_payload_{axes[i]}_time_movement_pd.svg', format='svg')

os.chdir('/home/felsager/Workspaces/auro_ws/src/ll_controller/csv')

class PDController:

    def __init__(self, waypoints):
        """Constructor """
        # Create ROS node 
        rospy.init_node(f'thrust_tuning')
        
        # Publisher for PX4 attitude offboard control
        self.attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10) 
        
        # Control callback necessary for keeping OFFBOARD on
        self.control_period = 0.01
        self.control_timer = rospy.Timer(rospy.Duration(self.control_period), self.control_callback)

        # Mavros pose and twist subscribers
        #self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback, queue_size=10) 
        self.velocity_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, self.velocity_callback, queue_size=10)
        
        # Gazebo ground truth subscriber of drone and payload state
        self.state_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_callback)

        # Arrays for saving time and states of drone and payload
        self.drone_states = []
        self.current_drone_state = [0, 0, 0]
        self.payload_states = []
        self.current_payload_state = [0, 0, 0]
        self.setpoints = []

        self.time = [0]

        # Arrays for saving variables needed in control
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)

        # PD coefficients
        self.kp_thrust = 1.5
        self.kd_thrust = -0.5

        self.kp_att = 0.5
        self.kd_att = -0.4

        # Waypoint initialization
        self.waypoints = np.array(waypoints)
        self.waypoint_no = waypoints.shape[0]
        self.waypoint_ind = 0
        self.current_waypoint = waypoints[0,:]

        # Error norm threshhold (error margin)
        self.error_threshold = 0.1

        # Change mode to offboard and arm the drone
        self.initialize_drone()
        print('Going to ', self.current_waypoint)      

    '''def pose_callback(self, data):
        """ Callback for drone pose subscriber """
        self.position[0] = data.pose.position.x
        self.position[1] = data.pose.position.y
        self.position[2] = data.pose.position.z
        q = data.pose.orientation
        self.orientation = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))
    '''

    def velocity_callback(self, data):
        """ Callback for drone velocity subscriber"""
        self.velocity[0] = data.twist.linear.x
        self.velocity[1] = data.twist.linear.y
        self.velocity[2] = data.twist.linear.z

    def update_waypoint(self):
        """ Increment waypoint index and set current waypoint"""
        self.waypoint_ind += 1
        if self.waypoint_ind >= self.waypoint_no:
            rospy.signal_shutdown('Last waypoint has been reached') # Problem with different threads not shutting down immediately 
        self.current_waypoint = self.waypoints[self.waypoint_ind]    

    def control_callback(self, event):
        """ Callback for control action """
        self.drone_states.append(self.current_drone_state[:])
        self.payload_states.append(self.current_payload_state[:])
        self.time.append(self.time[-1] + self.control_period)
        self.setpoints.append(self.waypoints[self.waypoint_ind])
        error, dist_error = self.get_position_error()
        if dist_error < self.error_threshold:
            if self.waypoint_ind == self.waypoint_no - 1:
                self.time = np.array(self.time)[:-251]
                self.drone_states = np.array(self.drone_states)[250:]
                self.payload_states = np.array(self.payload_states)[250:]
                self.setpoints = np.array(self.setpoints)[250:]
            self.update_waypoint()
            error, dist_error = self.get_position_error()
            print('Going to ', self.current_waypoint)
            print(f'{self.waypoint_ind = }')
        thrust = self.thrust_controller(error[2], \
            self.velocity[2], self.orientation[0], self.orientation[0])
        orientation = self.attitude_controller(error[0], error[1], \
             self.velocity[0], self.velocity[1])
        msg = AttitudeTarget()
        msg.type_mask = 3
        msg.thrust = thrust
        msg.orientation = orientation
        msg.body_rate.z = 0
        self.attitude_pub.publish(msg)

    def get_position_error(self):
        """ Calculate position error as vector from drone to desired waypoint. """
        error = np.subtract(self.current_waypoint, self.position)
        payload_error = np.subtract(self.current_drone_state, self.current_payload_state)
        payload_angle = np.arctan2(payload_error[2], payload_error[0]) - np.pi/2
        payload_angle = -np.arctan2(np.sin(payload_angle),np.cos(payload_angle))
        print(f'Orientation: {self.orientation}')
        print(f'Payload angle: {payload_angle}')
        print(f'Orientation error: {self.orientation[1] - payload_angle}')
        dist_err = np.linalg.norm(error[::2])
        return error, dist_err

    def thrust_controller(self, e_z, v_z, pitch, roll):
        """ PD position control of drone thrust. """
        angle_compensation = abs(np.cos(pitch)*np.cos(roll))
        if angle_compensation < 0.5: # compensate angles higher than pi/3 (saturation)
            angle_compensation = 0.5  
        thrust = 0.7 + self.kp_thrust*e_z + self.kd_thrust*v_z
        thrust /= angle_compensation
        return thrust

    def attitude_controller(self, e_x, e_y, v_x, v_y):
        """ PD position control of drone pitch and roll. """
        pitch = (self.kp_att*e_x + self.kd_att*v_x)
        roll = -(self.kp_att*e_y + self.kd_att*v_y)
        pitch = np.clip(pitch, -0.52, 0.52) # clip angles to range -pi/6, pi/6 (angles are not normalized)
        roll = np.clip(roll, -0.52, 0.52)
        roll = 0
        return Quaternion(*quaternion_from_euler(roll, pitch, 0))
    
    def state_callback(self, data):
        """ Update the current state of the drone """
        self.current_drone_state[0] = data.pose[1].position.x # index 1 is drone in gazebo
        self.current_drone_state[1] = data.pose[1].position.y
        self.current_drone_state[2] = data.pose[1].position.z
        self.current_payload_state[0] = data.pose[-1].position.x # index -1 (last) is payload in gazebo
        self.current_payload_state[1] = data.pose[-1].position.y
        self.current_payload_state[2] = data.pose[-1].position.z
        self.position = self.current_drone_state
        q = data.pose[1].orientation
        self.orientation = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))

    def initialize_drone(self):
        """ Set mode to "OFFBOARD" and arm the drone. """
        # Publish control setpoints to PX4 controller so mode can be change to "OFFBOARD"
        rate = rospy.Rate(500)
        for i in range(150):
            msg = AttitudeTarget()
            msg.type_mask = 7
            msg.thrust = 0
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
