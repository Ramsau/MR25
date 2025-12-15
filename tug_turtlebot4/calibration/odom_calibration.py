import numpy as np
import matplotlib
from matplotlib.axes import Axes

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


def read_pose_data(filename):
    data = np.loadtxt(filename)
    data = {
        'stamp': data[:, 0],
        'x': data[:, 1],
        'y': data[:, 2],
        'theta': data[:, 3]
    }
    return data


def read_odom_data(filename):
    data = np.loadtxt(filename)
    data = {
        'stamp': data[:, 0],
        'encoder_left': data[:, 1],
        'encoder_right': data[:, 2]
    }
    return data


def draw_arrow(ax: Axes, x: float, y: float, theta: float, length: float = 0.05, color='1'):
    dx = length * np.cos(theta)
    dy = length * np.sin(theta)
    ax.arrow(x, y, dx, dy, head_width=0.005, head_length=0.005, fc=color, ec='none')


def visualize_data(ax, pose_data, color='1'):
    ax.plot(pose_data['x'], pose_data['y'], color=color)

    # Draw arrows for every nth point to show orientation
    n = len(pose_data['x']) // 20  # Show ~20 arrows along the path
    if n < 1:
        n = 1
    for i in range(0, len(pose_data['x'])):
        draw_arrow(ax, pose_data['x'][i], pose_data['y'][i], pose_data['theta'][i], color=color)


def encoders_to_poses(encoder_data, init_x=0.0, init_y=0.0, init_yaw=0.0, ticks_per_rev=2573, wheel_diam=0.0715, wheel_base=0.233):
    wheel_circumference = wheel_diam * np.pi
    dist_left = encoder_data['encoder_left'] * wheel_circumference / ticks_per_rev
    dist_right = encoder_data['encoder_right'] * wheel_circumference / ticks_per_rev
    delta_s = (dist_left + dist_right) / 2
    delta_theta = (dist_right - dist_left) / wheel_base

    data = {
        'stamp': [0],
        'x': [init_x],
        'y': [init_y],
        'theta': [init_yaw],
    }

    for i in range(len(encoder_data['stamp'])):
        data['stamp'].append(encoder_data['stamp'][i])
        data['x'].append(data['x'][i] + delta_s[i] * np.cos(data['theta'][i] + delta_theta[i]))
        data['y'].append(data['y'][i] + delta_s[i] * np.sin(data['theta'][i] + delta_theta[i]))
        data['theta'].append(data['theta'][i] + delta_theta[i])

    return data

def main():
    # Read data files
    pose_data = read_pose_data('../../../../pose.dat')
    odom_data = read_odom_data('../../../../odom.dat')

    # Convert encoder data to pose data
    odom_pose_data_naive = encoders_to_poses(odom_data)

    # Visualize the data

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Robot Position')
    ax.grid(True)

    visualize_data(ax, pose_data, 'blue')
    visualize_data(ax, odom_pose_data_naive, 'red')

    plt.show()


if __name__ == '__main__':
    main()
