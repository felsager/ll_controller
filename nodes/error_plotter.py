import os
import numpy as np
import matplotlib.pyplot as plt

def error_plotter(e_x, e_y, e_swing, no, title):
    os.chdir('/home/felsager/Workspaces/auro_ws/src/ll_controller/plots')
    axes = ['x','z','\\theta']
    units = ['[m]','[m]','[rad]']
    titles = ['Error in $x$-position', 'Error in $z$-position', 'Error in swing angle']
    N = e_x.shape[0] - 5
    t = np.linspace(0, 0.01*N, N)
    fig, ax = plt.subplots(3)
    #ax.set_title(f'{title} errors vs time')
    ax[0].plot(t, 10*e_x[5:], color='red')
    ax[1].plot(t, e_y[5:], color='blue')
    ax[2].plot(t, np.pi*e_swing[5:], color='green')
    for i in range(3):
        ax[i].set_xlabel(f'$t$ [s]')
        ax[i].set_ylabel(f'$e_{axes[i]}$ {units[i]}')
        ax[i].set_title(f'{titles[i]}')
        ax[i].grid()
    fig.tight_layout()
    fig.savefig(f'errors_{no}_{title}.svg', format='svg')
