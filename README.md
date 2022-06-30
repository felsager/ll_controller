# Quadrotor Slung Payload Carrying using Model-Free Reinforcement Learning

The following guide assumes that you use Ubuntu 20.04, have Gazebo 11 and Robotic Operating System (ROS) Noetic installed: http://wiki.ros.org/noetic/Installation/Ubuntu

**Installation**
1. Clone this repository into a catkin workspace and build it
2. Install the Pixhawk 4 (PX4) Ubuntu Development Enviroment according to the following guide: https://docs.px4.io/v1.12/en/dev_setup/dev_env_linux_ubuntu.html#gazebo-jmavsim-and-nuttx-pixhawk-targets
3. In the installed PX4 repository delete the file "PX4-Autopilot/launch/posix_sitl.launch" and replace it with the file of the same name from this repository "ll_controller/px4_rl_payload/posix_sitl.launch"
4. Copy the folders "ll_controller/px4_rl_payload/cable", "ll_controller/px4_rl_payload/payload" and "ll_controller/px4_rl_payload/iris_load_test" into the folder "PX4-Autopilot/Tools/sitl_gazebo/models" 
5. Copy the file "ll_controller/px4_rl_payload/1080_iris_load_test" into the folder "PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes"
6. Replace the file "PX4-Autopilot/Tools/sitl_gazebo/worlds/empty.world" with the file "ll_controller/px4_rl_payload/empty.world"
7. Copy the file "ll_controller/px4_rl_payload/run.sh" into the folder "PX4-Autopilot"

**Running the simulation and Reinforcement Learning Controller and Training**
1. In a terminal change directory to "PX4-Autopilot" 
2. Run the following line: "bash run.sh"
3. In a new terminal run the following line "rostopic echo /gazebo/link_states/name" - if "goal_sphere::link" is last in the list of names change variable "idx_q" in "payload_env.py" to 1 and "idx_p" to -2 - if "goal_sphere::link" is the second upper name then the variables should be "idx_q=2" and "idx_p=-1"
4. In the terminal open a new window and change directory into this repository "ll_controller"
5. Run the Python file "run_env.py" with rosrun, which will run the final controller deterministic: "rosrun ll_controller run_env.py"
6. The Proportional Derivative controller can also be tested by changing "pd_control" to True in line 16 of "run_env.py"
7. If further training is wanted, increase the run_no, and change the variable "training" to True

Note:
The custom environment where tunable parameters such as reward shape ect. is the file "payload_env.py" 
