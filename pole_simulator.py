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
failure_angle = 0.5  # radians
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
A = np.array([[1, 1, 0, 0], [gravity * pole_length, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
B = np.array([[0], [-1], [0], [1]])
Q = 0.1 * np.diag([1 / (failure_angle ** 2), 1 / (0.3 ** 2), 1 / (track_limit ** 2), 1 / (5 ** 2)])
R = np.array([[1 / (20 ** 2)]])
# Calculate LQR optimal control policy
K, _, _ = control.lqr(A, B, Q, R)
# Example calculation
x = -1 * np.array([[0.1], [0], [3], [0]])
print(np.matmul(K, x))

import sys
sys.exit()

log = False

time_steps = 40_000
p = 136
d = 12
pos_p = -0.02
pos_d = -0.1
angle = 0.1
angle_speed = 0
pos = 3
pos_speed = 0
time = 0
abs_positions = []
for _ in range(time_steps):
    target_angle = (pos_p * pos) + (pos_d * pos_speed)
    p_error = angle - target_angle
    d_error = angle_speed
    force = (p_error * p) + (d_error * d)
    angle_accel = get_angle_accel(angle, angle_speed, force)
    pos_accel = get_pos_accel(angle, angle_speed, angle_accel, force)
    angle += angle_speed * time_step
    angle_speed += (angle_accel * time_step)
    pos += pos_speed * time_step
    abs_positions.append(abs(pos))
    pos_speed += pos_accel * time_step
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
    if abs(pos) > track_limit:
        if log:
            print('Failure')
        break
    time += time_step

print(np.mean(abs_positions))
plt.plot(abs_positions)
plt.show()
