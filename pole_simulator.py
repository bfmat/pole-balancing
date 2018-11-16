#!/usr/bin/env python3

import math

import control
import matplotlib.pyplot as plt
import numpy as np

gravity = 9.81
cart_mass = 1
pole_mass = 0.1
pole_length = 0.5
track_limit = 10
failure_angle = 1  # radians
time_step = 0.001


def get_angle_accel(angle, angle_speed, force):
    return (
        ((gravity * math.sin(angle)) + (math.cos(angle) * (
            (-force - (pole_mass * pole_length * (angle_speed ** 2) * math.sin(angle))) / (cart_mass + pole_mass)
        )))
        / (pole_length * ((4/3) - ((pole_mass * (math.cos(angle) ** 2)) / (cart_mass + pole_mass))))
    )


def get_pos_accel(angle, angle_speed, angle_accel, force):
    return (
        (force + (pole_mass * pole_length * (((angle_speed ** 2) * math.sin(angle)) + (angle_accel * math.cos(angle)))))
        / (cart_mass + pole_mass)
    )


# State space control matrices (linearized)
A = np.array([[1, 1, 0, 0], [gravity / pole_length, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
B = np.array([[0], [-1], [0], [1]])
Q = 0.001 * np.diag([1 / (0.1 ** 2), 1 / (0.1 ** 2), 1 / (20 ** 2), 1 / (1 ** 2)])
R = np.array([[1 / (5 ** 2)]])
# Calculate LQR optimal control policy
K, _, _ = control.lqr(A, B, Q, R)

log = False

time_steps = 20_000
angle = 0
angle_speed = 0
pos = 3
pos_speed = 0
time = 0

positions = []
angles = []
forces = []
times = []

for _ in range(time_steps):
    state = np.array([[angle], [angle_speed], [pos], [pos_speed]])
    force = np.matmul(K, (state * -1))[0, 0]
    angle_accel = get_angle_accel(angle, angle_speed, force)
    pos_accel = get_pos_accel(angle, angle_speed, angle_accel, force)
    angle += angle_speed * time_step
    angle_speed += (angle_accel * time_step)
    pos += pos_speed * time_step
    pos_speed += pos_accel * time_step
    time += time_step

    positions.append(pos)
    angles.append(angle)
    forces.append(force)
    times.append(time)

    if log:
        print('Time:', time)
        print('Force:', force)
        print('Position:', pos)
        print('Position Speed:', pos_speed)
        print('Position Acceleration:', pos_accel)
        print('Angle:', angle)
        print('Angle Speed:', angle_speed)
        print('Angle Acceleration:', angle_accel)
        print()
    if abs(pos) >= track_limit or abs(angle) >= failure_angle:
        if log:
            print('Failure')
        break

plt.subplot(3, 1, 1)
plt.plot(times, positions)
plt.title('Position')

plt.subplot(3, 1, 2)
plt.plot(times, angles)
plt.title('Angle')

plt.subplot(3, 1, 3)
plt.plot(times, forces)
plt.title('Force Applied')

plt.show()
