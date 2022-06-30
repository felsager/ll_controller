#!/usr/bin/env python

from payload_env import PayloadEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import SAC
import os
import numpy as np
import rospy
from error_plotter import*

run_no = 53

log_dir = f'/home/felsager/Documents/gym/{run_no}/'
os.makedirs(log_dir, exist_ok=True)
print(os.getcwd())
env = PayloadEnv(pd_control=False, infinite_goal=False)
env = Monitor(env, log_dir)

#model = SAC('MlpPolicy', env) # uncomment this line for fresh model and then comment out the next line
model = SAC.load('model/trained_model_new_obs_50_10', env=env) # model that is loaded in
training = False # train (True) or test (False)

if training:
    for i in range(1, 50):
        for j in range(10): # save more often
            model.learn(total_timesteps=int(1e4))
            model.save(f'model/trained_model_new_obs_{run_no}_{i}')

else:
    step_counter = 0
    total_swing_error = 0
    no = 1

    obs = env.reset()
    observations = np.copy(obs)
    start_time = rospy.get_time()
    for i in range(50000):
        observations = np.vstack((observations, obs))
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        total_swing_error += abs(obs[4])
        step_counter += 1
        if done:
            time_used = rospy.get_time() - start_time
            print(f'{no}: {time_used = }')
            # unnormalize erros
            avg_swing_error = total_swing_error/step_counter*np.pi
            print(f'{no}: {avg_swing_error}')
            print(f'{no}: {step_counter = }')
            print(f'{no}: {total_swing_error = }')
            #error_plotter(observations[:,0],observations[:,1],observations[:,4], no, 'PD Controller')
            no += 1
            step_counter = 0
            total_swing_error = 0
            start_time = rospy.get_time()
            obs = env.reset()
            observations = np.copy(obs)
