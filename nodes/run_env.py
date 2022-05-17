#!/usr/bin/env python

from payload_env import PayloadEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common import results_plotter
from stable_baselines3 import SAC

env = PayloadEnv(pd_control=False)
env = Monitor(env, 'learning_logs')

model = SAC('MlpPolicy', env)
#model = SAC.load('model/trained_model', env=env)

for i in range(10):
    model.learn(total_timesteps=int(1e5))
    model.save(f'model/trained_model2_{i}')
    results_plotter.plot_results(['learning_logs'], i*(1e5), results_plotter.X_TIMESTEPS)

'''
obs = env.reset()
for i in range(20000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()'''
    


