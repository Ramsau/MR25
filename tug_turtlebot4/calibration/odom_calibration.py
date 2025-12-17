import numpy as np
import matplotlib
from matplotlib.axes import Axes
from scipy.optimize import least_squares

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

TICKS_PER_REV = 2573

def read_pose_data(filename):
    data = np.loadtxt(filename)
    data = {
        'stamp': data[:, 0],
        'x': data[:, 1],
        'y': data[:, 2],
        'theta': data[:, 3]
    }
    # remap theta so it fits nicely to rest of data
    data['theta'][data['theta'] < -0.1] += 2 * np.pi
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


def visualize_poses(ax, pose_data, color='1'):
    ax.plot(pose_data['x'], pose_data['y'], color=color)

    # Draw arrows for every nth point to show orientation
    n = len(pose_data['x']) // 20  # Show ~20 arrows along the path
    if n < 1:
        n = 1
    for i in range(0, len(pose_data['x'])):
        draw_arrow(ax, pose_data['x'][i], pose_data['y'][i], pose_data['theta'][i], color=color)

def encoders_to_poses(encoder_data, init_x=0.0, init_y=0.0, init_yaw=0.0, wheel_diam_left=0.0715, wheel_diam_right=0.0715, wheel_base=0.233):
    global TICKS_PER_REV
    wheel_circumference_left = wheel_diam_left * np.pi
    wheel_circumference_right = wheel_diam_right * np.pi
    dist_left = encoder_data['encoder_left'] * wheel_circumference_left / TICKS_PER_REV
    dist_right = encoder_data['encoder_right'] * wheel_circumference_right / TICKS_PER_REV
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

    data['stamp'] = np.array(data['stamp'])
    data['x'] = np.array(data['x'])
    data['y'] = np.array(data['y'])
    data['theta'] = np.array(data['theta'])

    return data

def optimise_odometry(encoder_data, pose_data):
    time_diffs = np.diff(pose_data['stamp'], prepend=pose_data['stamp'][0]) * 1e-9

    wheel_speeds_left = encoder_data['encoder_left'] / TICKS_PER_REV * 2 * np.pi
    wheel_speeds_right = encoder_data['encoder_right'] / TICKS_PER_REV * 2 * np.pi

    rotational_speeds = np.diff(pose_data['theta'], prepend=pose_data['theta'][0])

    L = np.column_stack((wheel_speeds_left, wheel_speeds_right))

    # Least squares estimate of [J21, J22]
    J2, *_ = np.linalg.lstsq(L, rotational_speeds, rcond=None)

    v_l_est = lambda s: wheel_speeds_left * s[0]
    v_r_est = lambda s: wheel_speeds_right * s[1]
    v_est = lambda s: (v_l_est(s) + v_r_est(s)) / 2
    omega_est = lambda s: (v_r_est(s) - v_l_est(s)) / s[2]
    theta_est = lambda s: np.cumsum(omega_est(s))
    r_x = lambda s: v_est(s) * np.cos(theta_est(s))
    r_y = lambda s: v_est(s) * np.sin(theta_est(s))
    state_est = lambda s: np.column_stack((r_x(s), r_y(s), omega_est(s)))
    state_obs = np.column_stack((
        np.diff(pose_data['x'], prepend=pose_data['x'][0]),
        np.diff(pose_data['y'], prepend=pose_data['y'][0]),
        np.diff(pose_data['theta'], prepend=pose_data['theta'][0])
    ))
    e = lambda s: state_obs - state_est(s)

    # Perform least squares optimization
    result = least_squares(lambda s: e(s).ravel(), x0=np.array([0.0715/2, 0.0715/2, 0.233]))
    optimized_rl = result.x[0]
    optimized_rr = result.x[1]
    optimized_b = result.x[2]

    return optimized_rl, optimized_rr, optimized_b


def linear_interp(data_timeref, data_value):
    data_out = {'stamp': []}
    for key in data_value.keys():
        data_out[key] = np.interp(data_timeref['stamp'], data_value['stamp'], data_value[key])

    return data_out

def main():
    # Read data files
    pose_data = read_pose_data('../../../../pose.dat')
    odom_data = linear_interp(pose_data, read_odom_data('../../../../odom.dat'))

    # Convert encoder data to pose data
    odom_pose_data_naive = encoders_to_poses(odom_data)

    # Optimise odometry
    rl, rr, b = optimise_odometry(odom_data, pose_data)
    odom_pose_data_optimized = encoders_to_poses(odom_data, wheel_base=b, wheel_diam_left=rl*2, wheel_diam_right=rr*2)

    print(f"Optimized wheel base: {b}")
    print(f"Optimized wheel diameter left: {rl*2}")
    print(f"Optimized wheel diameter right: {rr*2}")

    # Visualize the data
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Position plot
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('Robot Position')
    ax1.grid(True)
    visualize_poses(ax1, pose_data, 'blue')
    visualize_poses(ax1, odom_pose_data_naive, 'red')
    visualize_poses(ax1, odom_pose_data_optimized, 'green')

    # Yaw angle plot
    ax2.set_ylabel('Yaw Angle')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True)
    ax2.plot(pose_data['stamp'] * 1e-9, pose_data['theta'], color='blue')
    ax2.plot(odom_pose_data_naive['stamp'] * 1e-9, odom_pose_data_naive['theta'], color='red')
    ax2.plot(odom_pose_data_optimized['stamp'] * 1e-9, odom_pose_data_optimized['theta'], color='green')
    ax2.legend(['Ground Truth', 'Odometry (naive)', 'Odometry (optimized)'])

    plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()
