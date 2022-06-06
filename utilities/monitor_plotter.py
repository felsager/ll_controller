import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3.common.results_plotter import load_results, ts2xy
plt.rcParams.update({'font.size': 13})

training_no = 39
log_dir = f"/home/felsager/Documents/gym/{training_no}/"

def moving_average(values, window):
    """
    Smooth values by doing a moving average
    :param values: (numpy array)
    :param window: (int)
    :return: (numpy array)
    """
    weights = np.repeat(1.0, window) / window
    return np.convolve(values, weights, 'valid')


def plot_results(log_folder, title='Learning Curve'):
    """
    plot the results

    :param log_folder: (str) the save location of the results to plot
    :param title: (str) the title of the task to plot
    """
    window_size = 1000
    x, y = ts2xy(load_results(log_folder), 'timesteps')
    y = moving_average(y, window=window_size) # longer training -> use bigger window to avoid high frequency noise
    # Truncate x
    x = x[len(x) - len(y):]
    fig = plt.figure(title)
    plt.plot(x, y, 'b', linewidth=2)
    plt.xlabel('Number of Timesteps')
    plt.ylabel('Rewards')
    plt.ylim([-200,0])
    plt.gca().set_aspect(1000)
    plt.title(title + " Smoothed")
    plt.legend([f'Filter size: {window_size}'], loc='lower right')
    plt.grid()
    fig.tight_layout()
    fig.savefig(f'learning_curve_{training_no}.svg', format='svg', bbox_inches="tight")
    plt.show()

plot_results(log_dir)