import rospy
from gazebo_msgs.srv import GetModelProperties, GetJointProperties

model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)


joints = model_properties('iris_load_test').joint_names[6:]

for joint in joints:
    joint_property = joint_properties(joint)
    print(joint_property)
    print(f'{joint}:\nPosition = {joint_property.position}\nVelocity = {joint_property.rate}\n')
    