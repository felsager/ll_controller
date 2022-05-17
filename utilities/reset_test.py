import rospy 
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
import subprocess

pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_model_poses = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

px4_ekf2_path = '/home/felsager/Documents/PX4-Autopilot/build/px4_sitl_default/bin/px4-ekf2'

joint_names = ['iris::rotor_0_joint', 'iris::rotor_1_joint', 'iris::rotor_2_joint', 'iris::rotor_3_joint', 'cable::link_1_to_link_2', 'cable::link_2_to_link_3', 'cable::link_3_to_link_4', 'cable::link_4_to_link_5', 'cable::link_5_to_link_6', 'cable::link_6_to_link_7', 'cable::link_7_to_link_8', 'cable::link_8_to_link_9', 'cable::link_9_to_link_10', 'iris_cable_joint','cable_payload_joint']
joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def set_state(x, z):
    state_msg = ModelState()
    state_msg.model_name = 'iris_load_test'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 5
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
    resp = reset_model_poses(state_msg)
    print(resp)

def reset():
    pause_physics()
    for i in range(3):
        reset_joints(model_name = "iris_load_test", joint_names = joint_names, joint_positions = joint_positions)
        full_state = get_state(model_name='iris_load_test').pose.position
        set_state(full_state.x, full_state.z)
    #rospy.sleep(0.01)
    #unpause_physics()
    #ekf2_stop = subprocess.Popen([px4_ekf2_path, "stop"])
    #rospy.sleep(1)
    #ekf2_start = subprocess.Popen([px4_ekf2_path, "start"])
    #subprocess.Popen([px4_ekf2_path, "status"])
    #rospy.sleep(1)

reset()