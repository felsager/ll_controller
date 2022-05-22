#!/usr/bin/env python

import gym
from gym import spaces
import torch
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TwistStamped, Quaternion
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import AttitudeTarget
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import subprocess
import time

class PayloadEnv(gym.Env):
    def __init__(self, pd_control):
        ''' Initialize GYM environment. '''
        rospy.init_node(f'env_node')
        
        # Publisher for PX4 attitude offboard control
        self.attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        # Gazebo ground truth subscriber of drone and payload state
        self.gazebo_state_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_state_callback)
        self.v_x = 0
        self.v_z = 0
        self.x_normalize = 10 # position normalization factor in x
        self.z_normalize = 1 # position normalization factor in z
        self.v_x_normalize = 5 # velocity normalization factor in x
        self.v_z_normalize = 1 # velocity normalization factor in z

        self.norm_normalize = 1

        self.state = np.zeros(7, dtype=np.float32) # saving state for PD controller

        # Initial control action
        self.thrust = 0.7
        self.pitch = 0
        self.pd_control = pd_control # boolean for switching between RL and PD controller

        # Reset environment utilities
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.reset_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.px4_ekf2_path = '/home/felsager/Documents/PX4-Autopilot/build/px4_sitl_default/bin/px4-ekf2'

        self.joint_names = ['cable::link_1_to_link_2', 'cable::link_2_to_link_3', 'cable::link_3_to_link_4', 'cable::link_4_to_link_5', 'cable::link_5_to_link_6', 'cable::link_6_to_link_7', 'cable::link_7_to_link_8', 'cable::link_8_to_link_9', 'cable::link_9_to_link_10', 'iris_cable_joint','cable_payload_joint']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Arrays for saving time and states of drone and payload
        self.drone_states = []
        self.current_drone_state = [0, 0]
        self.payload_states = []
        self.current_payload_state = [0, 0]
        self.drone_pitch = 0
        self.drone_yaw = 0
        self.drone_pitch_normalized = 0
        self.desired_position = np.zeros(2)
        
        # Reward coefficients - should add up to two (they are already normalized and the reward range should be [-1 to 1])
        self.k_alpha = 0.7 # position error reward coefficient
        self.k_beta = 0.07 # velocity reward coefficient - velocity is not normalized and should have a lower coefficient?
        self.k_theta = 0.3 # theta error reward coefficient
        self.k_delta = 0.1 # theta error dot reward coefficient
        self.k_gamma = 0.06 # pitch angle reward coefficient

        self.pre_e_theta = 0
        self.e_theta_dot = 0
        self.step_counter = 1

        self.step_size = 0.01
        rate = int(1/self.step_size)

        # Variable for monitoring time per episode
        self.start_time = 0
        self.rate = rospy.Rate(rate)
        for i in range(3): # send multiple times to ensure it resets all
            self.reset_joints(model_name = "iris_load_test", \
                joint_names = self.joint_names, \
                    joint_positions = self.joint_positions)
            self.set_state(0, 0)
        # Initalizing the RL action and observation space - normalized??
        self.action_space = spaces.Box(low=np.array([-1, -1], dtype=np.float32), high=np.array([1, 1], dtype=np.float32)) # thrust and pitch normalized to [-1, 1]
        self.observation_space = spaces.Box(low=np.array([-1, -1, -1, -1, -1, -1, -1], dtype=np.float32), high=np.array([1, 1, 1, 1, 1, 1, 1], dtype=np.float32)) # e_x, e_z, v_x, v_z, e_theta, e_theta_dot, pitch angle of drone
        print('Initializing drone: ')
        self.control_timer = rospy.Timer(rospy.Duration(0.001), self.control_callback)
        self.initialize_drone()

    def step(self, action):
        ''' Take step in environment. '''
        self.set_action(action) 
        state = self.calculate_state()
        self.state = state
        normed_pos_error = np.linalg.norm(state[:2])
        normed_velocity = np.linalg.norm(state[2:4])
        reward = self.calculate_reward(state)
        print(f'{state = }')
        print(f'{reward = }')
        time_used = rospy.get_time() - self.start_time
        # when should the episode finish and reset
        if normed_pos_error < 0.1 and normed_velocity < 0.25: # changed the normed velocity requirment from 0.1 to 0.25 - quadrotor reached goal and kept oscillating because of steady state velocity 
            done = True
            reward += 150 - 2*time_used/self.norm_normalize # more reward for higher velocity
        elif time_used > 25: # changed time stop from 50 -> 20 so reward is not just accumalated
            done = True
            reward -= 100 
        elif self.v_z < -6 or self.current_drone_state[1] < 2 or abs(self.drone_yaw) > 0.2  or normed_pos_error > 30:
            done = True
            reward -= 150 # stronger punishment for just falling to the ground or yaw drift - inaccurate rope model
        else:
            done = False
        info = {} #[f'{self.desired_position, self.current_drone_state = }'] # placeholder
        self.rate.sleep()
        return state, reward, done, info

    def reset(self):
        ''' Reset environment - resets the rope and makes the drone have zero velocity and a new goal at a given distance from its current position'''
        self.thrust = 0.7 # set thrust higher to stabilize?
        self.pitch = 0
        for i in range(3): # send multiple times to ensure it resets all
            self.set_state(0, 20)
            self.reset_joints(model_name = "iris_load_test", \
                joint_names = self.joint_names, \
                    joint_positions = self.joint_positions)

        self.current_drone_state = [0, 20]
        # generate random side of both x and z where the desired position is placed 
        x_des = np.random.uniform(-10, 10) # avoiding overfitting
        z_des = 20 + np.random.uniform(-0.5, 0.5) # +20 bias for same height as drone starts in
        self.x_normalize = abs(x_des)
        self.z_normalize = abs(z_des)
        self.norm_normalize = np.sqrt(self.x_normalize**2+self.z_normalize**2)
        self.desired_position = [x_des, z_des]
        state = self.calculate_state()
        self.pre_e_theta = 0
        self.drone_pitch = 0
        self.drone_pitch_normalized = 0
        self.drone_yaw = 0
        self.start_time = rospy.get_time()
        return state
    
    def calculate_state(self):
        ''' Calculates the state used by the RL algorithm (normalized) [e_x, e_y, e_theta]. '''
        e_x = self.desired_position[0] - self.current_drone_state[0]
        e_z = self.desired_position[1] - self.current_drone_state[1]
        payload_vec = np.subtract(self.current_drone_state, self.current_payload_state)
        theta_payload = np.arctan2(payload_vec[1], payload_vec[0]) - np.pi/2
        theta_payload = -np.arctan2(np.sin(theta_payload),np.cos(theta_payload))
        e_theta = self.drone_pitch - theta_payload
        e_x /= self.x_normalize # normalize to maximum desired distance
        e_z /= self.z_normalize
        e_theta /= np.pi # normalize angle
        if e_theta == self.pre_e_theta:
            self.step_counter += 1
        else:
            self.e_theta_dot = (e_theta - self.pre_e_theta)/(2*self.step_size*self.step_counter) # doesn't need to be normalized - e_theta is already normalized
            self.pre_e_theta = e_theta
            self.step_counter = 1
        v_x_normalized = self.v_x/self.v_x_normalize
        v_z_normalized = self.v_z/self.v_z_normalize
        return np.array([e_x, e_z, v_x_normalized, v_z_normalized, e_theta, self.e_theta_dot, self.drone_pitch_normalized], dtype=np.float32)

    def calculate_reward(self, state):
        """ Calculate the reward. """
        reward = 1 - self.k_alpha*(abs(state[0]) + abs(state[1])) - self.k_beta*(abs(state[2]) + abs(state[3])) - self.k_theta*abs(state[4]) - self.k_delta*abs(state[5]) - self.k_gamma*abs(state[6])
        return reward

    def set_state(self, x, z):
        """ Resets the state of the drone to same position with zero joint angles and zeros velocities"""
        state_msg = ModelState()
        state_msg.model_name = 'iris_load_test'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1
        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0
        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0
        self.reset_model_state(state_msg)
    
    def set_action(self, action):
        if not self.pd_control:
            self.thrust = 0.7 + 0.3*action[0] # maps thrust from [-1, 1] to [0.4, 1] where 0 maps to 0.7 which is hover equilbrium
            self.pitch = 0.52*action[1] # scales pitch to interval [-pi/6,pi/6]
        else:
            angle_compensation = abs(np.cos(self.pitch))
            if angle_compensation < 0.5: # compensate angles higher than pi/3 (saturation)
                angle_compensation = 0.5  
            self.thrust = 0.7 + (1.5*self.state[1]*self.z_normalize - 0.5*self.state[3])/angle_compensation # coefficients from pd_controller node - use unnormalized errors
            self.pitch = 0.5*self.state[0]*self.x_normalize - 0.4*self.state[2]
            self.pitch = np.clip(self.pitch, -0.52, 0.52) # clip angles to range -pi/6, pi/6 (angles are not normalized)
        self.orientation = Quaternion(*quaternion_from_euler(0, self.pitch, 0))
    
    def control_callback(self, event):
        msg = AttitudeTarget()
        msg.thrust = self.thrust
        msg.type_mask = 7
        q = Quaternion(*quaternion_from_euler(0, self.pitch, 0))
        msg.orientation = q
        msg.body_rate.z = 0
        self.attitude_pub.publish(msg)

    def gazebo_state_callback(self, data):
        """ Update the current state of the drone """
        self.current_drone_state[0] = data.pose[1].position.x # index 1 is drone in gazebo
        self.current_drone_state[1] = data.pose[1].position.z
        self.current_payload_state[0] = data.pose[-1].position.x # index -1 (last) is payload in gazebo
        self.current_payload_state[1] = data.pose[-1].position.z
        self.v_x = data.twist[1].linear.x
        self.v_z = data.twist[1].linear.z
        q = data.pose[1].orientation
        orientation = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))
        self.drone_pitch = orientation[1]
        self.drone_yaw = orientation[2]
        self.drone_pitch_normalized = self.drone_pitch/np.pi

    def initialize_drone(self):
        """ Set mode to "OFFBOARD" and arm the drone. """
        # Publish control setpoints to PX4 controller so mode can be change to "OFFBOARD"
        init_rate = rospy.Rate(50)
        self.unpause_physics()
        for i in range(300):
            msg = AttitudeTarget()
            msg.type_mask = 7
            msg.thrust = self.thrust
            msg.orientation = Quaternion(*quaternion_from_euler(0, self.pitch, 0))
            self.attitude_pub.publish(msg)
            init_rate.sleep()
        
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
            #armed = response == 'success: False'
            #print(f'{armed = }')
        except rospy.ServiceException as e:
            print("Arming failed: %s" %e)
        
