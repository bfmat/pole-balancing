#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np

gravity = 9.81
cart_mass = 1
pole_mass = 0.1
pole_length = 0.5
track_limit = 2.4
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


log = True

p_range = 200
d_range = 200
target_angle = -0.3
results = []
parameters = []
angles = []
time_steps = 40_000
for p in [136]:
    print(p)
    for d in [12]:
        angle = -0.1
        angle_speed = 0
        pos = 0
        pos_speed = 0
        force = 0
        time = 0
        e = []
        for _ in range(time_steps):
            p_error = angle - target_angle
            d_error = angle_speed
            e.append(abs(p_error))
            force = (p_error * p) + (d_error * d)
            angle_accel = get_angle_accel(angle, angle_speed, force)
            pos_accel = get_pos_accel(angle, angle_speed, angle_accel, force)
            angle += angle_speed * time_step
            angles.append(angle)
            angle_speed += (angle_accel * time_step)
            pos += pos_speed * time_step
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
            if abs(angle) > failure_angle:  # or abs(pos) > track_limit:
                if log:
                    print('Failure')
                break
            time += time_step
        if len(e) == time_steps:
            results.append(np.mean(e))
            parameters.append((p, d))

# print(min(results))
# print(parameters[results.index(min(results))])
plt.plot(angles)
plt.show()
