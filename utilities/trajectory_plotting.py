import numpy as np
import matplotlib.pyplot as plt
import os

task_1 = ['no', 'payload']
task_2 = ['attached', 'payload']
task_3 = ['slung', 'payload']

# np = no payload, at = attached payload, sp = slung payload
os.chdir('/home/felsager/Workspaces/auro_ws/src/ll_controller/csv')

drone_states_np = np.genfromtxt(f'{task_1[0]}_{task_1[1]}_drone_state.csv', delimiter=',')
drone_states_ap = np.genfromtxt(f'{task_2[0]}_{task_2[1]}_drone_state.csv', delimiter=',')
drone_states_sp = np.genfromtxt(f'{task_3[0]}_{task_3[1]}_drone_state.csv', delimiter=',')

payload_states = np.genfromtxt(f'{task_3[0]}_{task_3[1]}_payload_state.csv', delimiter=',')

time_1 = np.genfromtxt(f'{task_1[0]}_{task_1[1]}_time.csv', delimiter=',')
time_2 = np.genfromtxt(f'{task_2[0]}_{task_2[1]}_time.csv', delimiter=',')
time_3 = np.genfromtxt(f'{task_3[0]}_{task_3[1]}_time.csv', delimiter=',')

os.chdir('/home/felsager/Workspaces/auro_ws/src/ll_controller/plots')

xd_np = drone_states_np[:,0]
yd_np = drone_states_np[:,1]
zd_np = drone_states_np[:,2]

xd_ap = drone_states_ap[:,0]
yd_ap = drone_states_ap[:,1]
zd_ap = drone_states_ap[:,2]

xd_sp = drone_states_sp[:,0]
yd_sp = drone_states_sp[:,1]
zd_sp = drone_states_sp[:,2]

xp = payload_states[:,0]
yp = payload_states[:,1]
zp = payload_states[:,2]

waypoints_xy = [[0, 0],
                [10, 0],
                [0, 0]]
waypoints_xz = [[0, 3],
                [10, 3],
                [0, 3]]

def trajectory_plotter(x, y, axes, thr, waypoints, task, ylim, xp = [0], yp = [0]):
    fig, ax = plt.subplots()
    theta = np.linspace(0,2*np.pi,200)
    for point in waypoints:
        x_t = point[0]; y_t = point[1]
        ax.plot(x_t, y_t, 'o', color='black', mfc='none',
              label='Waypoint: ' + str(point))

        ax.plot(np.ones(200)*x_t + thr*np.cos(theta),
                np.ones(200)*y_t + thr*np.sin(theta),
                ':', linewidth=2, color='black')

    ax.plot(x, y, color='red', label='Drone trajectory')
    if not (xp[0] == 0):
      ax.plot(xp, yp, ':', color='blue', label='Payload trajectory')

    ax.set_title(f'Trajectory of drone with {task[0]} {task[1]} in {axes}')
    ax.set_aspect('equal', 'box')
    ax.set_xlabel(axes[0])
    ax.set_ylabel(axes[1])
    ax.set_ylim(ylim)
    ax.legend(loc='lower center', fancybox=True)
    ax.grid()
    fig.tight_layout()
    fig.savefig(f'{task[0]}_{task[1]}_{axes}.svg', format='svg')

def time_plotter(t, drone_states, task, payload_states = [[0]]):
    fig, ax = plt.subplots(nrows=3, ncols=1) 
    axes = 'xyz'
    for i in range(3):
        if i == 1:
            ax[i].set_title(f'{(task[0]).title()} {task[1]} axes movement as functions of time')
        ax[i].plot(t, drone_states[:,i], color='red', label=f'Drone {axes[i]} movement')
        if not (payload_states[0][0] == 0):
            ax[i].plot(t, payload_states[:,i], color='blue', label=f'Payload {axes[i]} movement')
        ax[i].set_xlabel(f't [s]')
        ax[i].set_ylabel(f'{axes[i]} [m]')
        ax[i].legend(loc='upper left', fancybox=True)
        ax[i].grid()
    fig.tight_layout()
    fig.savefig(f'{task[0]}_{task[1]}_time_movement.svg', format='svg')

def time_together_plotter(t_1, t_2, t_3, drone_states_1, drone_states_2, drone_states_3, payload_states): 
    axes = 'xyz'
    for i in range(3):
        fig, ax = plt.subplots()
        ax.set_title(f'All scenarios {axes[i]} plotted against time')
        ax.plot(t_1, drone_states_1[:,i], color='red', label=f'No payload drone {axes[i]} movement')
        ax.plot(t_2, drone_states_2[:,i], color='green', label=f'Attached payload drone {axes[i]} movement')
        ax.plot(t_3, drone_states_3[:,i], color='orange', label=f'Slung payload drone {axes[i]} movement')
        ax.plot(t_3, payload_states[:,i], ':', color='blue', label=f'Slung payload {axes[i]} movement')
        ax.legend(loc='upper left', fancybox=True)
        ax.set_xlabel(f't [s]')
        ax.set_ylabel(f'{axes[i]} [m]')
        ax.grid()
        fig.tight_layout()
        fig.savefig(f'{axes[i]}_time_movement_all.svg', format='svg')

# create plots
trajectory_plotter(xd_np, yd_np, 'xy', 0.3, waypoints_xy, task_1, [-4, 1])
trajectory_plotter(xd_np, zd_np, 'xz', 0.3, waypoints_xz, task_1, [0, 5])
trajectory_plotter(xd_ap, yd_ap, 'xy', 0.3, waypoints_xy, task_2, [-4, 1])
trajectory_plotter(xd_ap, zd_ap, 'xz', 0.3, waypoints_xz, task_2, [0, 5])
trajectory_plotter(xd_sp, yd_sp, 'xy', 0.3, waypoints_xy, task_3, [-4, 1], xp, yp)
trajectory_plotter(xd_sp, zd_sp, 'xz', 0.3, waypoints_xz, task_3, [0, 5], xp, zp)
time_plotter(time_1, drone_states_np, task_1)
time_plotter(time_2, drone_states_ap, task_2)
time_plotter(time_3, drone_states_sp, task_3)
time_together_plotter(time_1, time_2, time_3, drone_states_np, drone_states_ap, drone_states_sp, payload_states)
