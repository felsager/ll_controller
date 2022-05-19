#!/usr/bin/env python

from payload_env import PayloadEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import SAC
import os

log_dir = "/tmp/gym/"
os.makedirs(log_dir, exist_ok=True)

env = PayloadEnv(pd_control=False)
env = Monitor(env, log_dir)

#model = SAC('MlpPolicy', env)
model = SAC.load('model/trained_model4_10', env=env)


for i in range(1, 5):
    for j in range(10): # save more often
        model.learn(total_timesteps=int(1e4))
        model.save(f'model/trained_model5_{i}')
'''

obs = env.reset()
for i in range(20000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    print(info)
    if done:
        obs = env.reset()
'''


