import rospy
rospy.init_node('test')

rospy.sleep(0.1)
time = rospy.get_time()
time_dif = 0
while time_dif < 100: 
    time_dif = rospy.get_time() - time
    print(time_dif)