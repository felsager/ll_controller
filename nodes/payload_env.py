#!/usr/bin/env python

import gym
from gym import spaces
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import AttitudeTarget
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState, LinkStates

class PayloadEnv(gym.Env):
    def __init__(self, pd_control, infinite_goal):
        ''' Initialize GYM environment. '''
        rospy.init_node(f'env_node')
        
        # Publisher for PX4 attitude offboard control
        self.attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        # Gazebo ground truth subscriber of drone and payload state
        self.gazebo_state_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates, self.gazebo_state_callback)
        self.v_x = 0
        self.v_y = 0
        self.v_z = 0
        self.x_normalize = 10 # position normalization factor in x
        self.y_normalize = 10
        self.z_normalize = 1 # position normalization factor in z
        self.v_x_normalize = 5 # velocity normalization factor in x - about max velocity it achieves
        self.v_y_normalize = 5
        self.v_z_normalize = 5 # velocity normalization factor in z

        self.e_y = 0

        self.norm_normalize = 1

        self.state = np.zeros(7, dtype=np.float32) # saving state for PD controller

        # Initial control action
        self.thrust = 0.7
        self.pitch = 0
        self.roll = 0
        self.pd_control = pd_control # boolean for switching between RL and PD controller
        self.infinite_goal = infinite_goal

        self.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

        # Reset environment utilities
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.reset_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.px4_ekf2_path = '/home/felsager/Documents/PX4-Autopilot/build/px4_sitl_default/bin/px4-ekf2'

        self.joint_names = ['cable::link_1_to_link_2', 'cable::link_2_to_link_3', 'cable::link_3_to_link_4', 'cable::link_4_to_link_5', 'cable::link_5_to_link_6', 'cable::link_6_to_link_7', 'cable::link_7_to_link_8', 'cable::link_8_to_link_9', 'cable::link_9_to_link_10', 'cable::link_10_to_link_11', 'cable::link_11_to_link_12', 'cable::link_12_to_link_13', 'cable::link_13_to_link_14', 'cable::link_14_to_link_15', 'cable::link_15_to_link_16', 'cable::link_16_to_link_17', 'cable::link_17_to_link_18', 'cable::link_18_to_link_19', 'cable::link_19_to_link_20', 'iris_cable_joint','cable_payload_joint']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Arrays for saving time and states of drone and payload
        self.drone_states = []
        self.current_drone_state = [0, 0, 0]
        self.payload_states = []
        self.current_payload_state = [0, 0, 0]
        self.drone_pitch = 0
        self.drone_roll = 0
        self.drone_yaw = 0
        self.drone_pitch_normalized = 0
        self.drone_roll_normalized = 0
        self.desired_position = np.zeros(2)
        
        # Reward coefficients - should add up to two (they are already normalized and the reward range should be [-1 to 1])
        self.k_alpha = 0.9 # position x error reward coefficient
        self.k_alpha_z = 0.7 # position z error reward coefficient
        self.k_beta = 0.05 # velocity xz reward coefficient - velocity is not normalized and should have a lower coefficient?
        self.k_angle = 0.2 # swing angle error reward coefficient
        self.k_delta = 0.05 # swing error dot
        self.k_gamma = 0.05 # pitch angle reward coefficient

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
            self.set_state(0, 0, 0)
        self.move_goal_sphere(0, 0, 20)
        # Initalizing the RL action and observation space - normalized??
        self.action_space = spaces.Box(low=np.array([-1, -1], dtype=np.float32), high=np.array([1, 1], dtype=np.float32)) # thrust, pitch normalized to [-1, 1]
        self.observation_space = spaces.Box(low=np.array([-1, -1, -1, -1, -1, -1, -1], dtype=np.float32), high=np.array([1, 1, 1, 1, 1, 1, 1], dtype=np.float32)) # e_x, e_z, v_x,  v_z, e_theta, e_theta_dot, theta 
        print('Initializing drone: ')
        self.control_timer = rospy.Timer(rospy.Duration(0.001), self.control_callback)
        self.initialize_drone()

    def step(self, action):
        ''' Take step in environment. '''
        #if self.infinite_goal:
        #    self.move_desired_position()
        self.set_action(action) 
        state = self.calculate_state()
        self.state = state
        print(state)
        normed_pos_error = np.linalg.norm([state[0]*self.x_normalize, state[1]]) # not normalized
        normed_velocity = np.linalg.norm(state[2:4]) # normalized - is this okay?
        reward = self.calculate_reward(state)
        time_used = rospy.get_time() - self.start_time
        # when should the episode finish and reset'
        if normed_pos_error < 0.05 and normed_velocity < 0.05 and self.drone_pitch < 0.05 and time_used > 0.2: # changed the normed velocity requirement from 0.1 to 0.25 - quadrotor reached goal and kept oscillating because of steady state velocity 
            done = True
            vel_punish = 400*time_used/self.norm_normalize
            if vel_punish > 300:
                vel_punish = 300
            reward += 400 - vel_punish # more reward for higher velocity
        elif time_used > 35: # changed time stop from 50 -> 20 so reward is not just accumalated
            done = True
            reward -= 150 
        elif self.v_z < -6 or self.current_drone_state[2] < 5:
            done = True
            reward -= 30
        else:
            done = False
        info = {} #[f'{self.desired_position, self.current_drone_state = }'] # placeholder
        self.rate.sleep()
        return state, reward, done, info

    def reset(self):
        ''' Reset environment - resets the rope and makes the drone have zero velocity and a new goal at a given distance from its current position'''
        self.thrust = 0.7 # set thrust higher to stabilize?
        self.pitch = 0
        self.roll = 0
        for i in range(3): # send multiple times to ensure it resets all
            self.set_state(0, 0, 19.95)
            self.reset_joints(model_name = "iris_load_test", \
                joint_names = self.joint_names, \
                    joint_positions = self.joint_positions)

        self.current_drone_state = [0, 0, 20]
        y_des = 0
        self.e_y = 0
        if not self.infinite_goal:
            # generate random side of both x and z where the desired position is placed 
            x_des = 10 #*np.power(-1, np.random.randint(2)) # variance in x
            z_des = 20 #+ np.random.uniform(-1, 1) # +20 bias for same height as drone starts in - variance in z

        else:
            x_des = 10
            z_des = 20
        self.move_goal_sphere(x_des, y_des, z_des)
        self.x_normalize = 10 # maximum desired position used for normalization
        self.z_normalize = 1
        self.norm_normalize = abs(x_des)
        self.desired_position = [x_des, z_des]
        state = self.calculate_state()
        self.pre_e_theta = 0
        self.drone_pitch = 0
        self.drone_roll = 0
        self.drone_pitch_normalized = 0
        self.drone_roll_normalized = 0
        self.drone_yaw = 0
        self.start_time = rospy.get_time()
        return state
    
    def calculate_state(self):
        ''' Calculates the state used by the RL algorithm (normalized) [e_x, e_y, e_theta]. '''
        e_x = self.desired_position[0] - self.current_drone_state[0]
        self.e_y = -self.current_drone_state[1]
        e_z = self.desired_position[-1] - self.current_drone_state[2]
        payload_vec = np.subtract(self.current_drone_state, self.current_payload_state)
        theta_payload = np.arctan2(payload_vec[2], payload_vec[0]) - np.pi/2
        theta_payload = -np.arctan2(np.sin(theta_payload),np.cos(theta_payload))
        e_theta = self.drone_pitch - theta_payload
        e_x /= self.x_normalize # normalize to maximum desired distance
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
        reward = 1 - self.k_alpha*abs(state[0]) - self.k_alpha_z*abs(state[1]) - self.k_beta*(abs(state[2]) + abs(state[3])) - self.k_angle*abs(state[4]) - self.k_delta*abs(state[5])- self.k_gamma*abs(state[6])
        return reward

    def set_state(self, x, y, z):
        """ Resets the state of the drone to same position with zero joint angles and zeros velocities. """
        state_msg = ModelState()
        state_msg.model_name = 'iris_load_test'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
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

    def move_goal_sphere(self, x_des, y_des, z_des):
        """ Sets the state of the goal sphere to the desired position. """
        state_msg = ModelState()
        state_msg.model_name = 'goal_sphere'
        state_msg.pose.position.x = x_des
        state_msg.pose.position.y = y_des
        state_msg.pose.position.z = z_des
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
        angle_compensation = abs(np.cos(self.drone_pitch)*np.cos(self.drone_roll))
        if angle_compensation < 0.5: # compensate angles higher than pi/3 (saturation)
            angle_compensation = 0.5 
        if not self.pd_control:
            self.thrust = 0.83 + 0.17*action[0] # maps thrust from [-1, 1] to [0.4, 1] where 0 maps to 0.7 which is hover equilbrium
            self.pitch = 0.31*action[1] # range from -pi/6 to pi/6
            self.roll = 0.6*self.current_drone_state[1] + 0.3*self.v_y
        else:
            self.thrust = (0.83 + (1.5*self.state[1] - 0.5*self.v_z))/angle_compensation # coefficients from pd_controller node - use unnormalized errors
            self.pitch = 0.6*self.state[0]*self.x_normalize - 0.3*self.v_x
            self.roll = 0.6*self.current_drone_state[1] + 0.3*self.v_y
            self.pitch = np.clip(self.pitch, -0.31, 0.31) # clip angles to avoid flipping over
        self.roll = np.clip(self.roll, -0.31, 0.31)
        self.orientation = Quaternion(*quaternion_from_euler(self.roll, self.pitch, 0))
    
    def control_callback(self, event):
        msg = AttitudeTarget()
        msg.thrust = self.thrust
        msg.type_mask = 3
        msg.body_rate.z = 0
        msg.orientation = self.orientation
        self.attitude_pub.publish(msg)

    def move_desired_position(self):
        x_des = 10 + self.current_drone_state[0]
        y_des = 0
        z_des = 20
        self.move_goal_sphere(x_des, y_des, z_des)

    def gazebo_state_callback(self, data):
        """ Update the current state of the drone """
        idx_q = 2
        idx_p = -1
        self.current_drone_state[0] = data.pose[idx_q].position.x 
        self.current_drone_state[1] = data.pose[idx_q].position.y
        self.current_drone_state[2] = data.pose[idx_q].position.z
        self.current_payload_state[0] = data.pose[idx_p].position.x # index -1 (last) is payload in gazebo
        self.current_payload_state[1] = data.pose[idx_p].position.y
        self.current_payload_state[2] = data.pose[idx_p].position.z
        self.v_x = data.twist[idx_q].linear.x
        self.v_y = data.twist[idx_q].linear.y
        self.v_z = data.twist[idx_q].linear.z
        q = data.pose[idx_q].orientation
        orientation = np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))
        self.drone_roll = orientation[0]
        self.drone_pitch = orientation[1]
        self.drone_yaw = orientation[2]
        self.drone_roll_normalized = self.drone_roll/np.pi
        self.drone_pitch_normalized = self.drone_pitch/np.pi

    def initialize_drone(self):
        """ Set mode to "OFFBOARD" and arm the drone. """
        # Publish control setpoints to PX4 controller so mode can be change to "OFFBOARD"
        init_rate = rospy.Rate(50)
        self.unpause_physics()
        for i in range(300):
            msg = AttitudeTarget()
            msg.type_mask = 4
            msg.thrust = self.thrust
            msg.body_rate.z = 0
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
        
