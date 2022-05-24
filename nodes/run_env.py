#!/usr/bin/env python

from payload_env import PayloadEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import SAC
import os

run_no = 8

log_dir = f'/home/felsager/Documents/gym/{run_no}/'
os.makedirs(log_dir, exist_ok=True)

env = PayloadEnv(pd_control=False, infinite_goal=False)
env = Monitor(env, log_dir)

#model = SAC('MlpPolicy', env)
model = SAC.load('model/trained_model_new_obs_7_4', env=env)


for i in range(1, 50):
    for j in range(10): # save more often
        model.learn(total_timesteps=int(1e4))
        model.save(f'model/trained_model_new_obs_{run_no}_{i}')

'''
obs = env.reset()
for i in range(30000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    print(info)
    if done:
        obs = env.reset()
'''


