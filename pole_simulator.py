#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np

gravity = 9.81
cart_mass = 1
pole_mass = 0.1
pole_length = 0.5
track_limit = 2.4
failure_angle = 0.209  # radians
time_step = 0.02


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


x = np.zeros((100, 100))
for p in range(100):
    for d in range(100):
        angle = 0.01
        angle_speed = 0
        pos = 0
        pos_speed = 0
        force = 0
        time = 0
        for _ in range(20000):
            force = angle * p + angle_speed * d
            angle_accel = get_angle_accel(angle, angle_speed, force)
            pos_accel = get_pos_accel(angle, angle_speed, angle_accel, force)
            angle += angle_speed * time_step
            angle_speed += (angle_accel * time_step)
            pos += pos_speed * time_step
            pos_speed += pos_accel * time_step
            # print('Time:', time)
            # print('Force:', force)
            # print('Position:', pos)
            # print('Position Speed:', pos_speed)
            # print('Position Acceleration:', pos_accel)
            # print('Angle:', angle)
            # print('Angle Speed:', angle_speed)
            # print('Angle Acceleration:', angle_accel)
            # print()
            if abs(angle) > failure_angle or abs(pos) > track_limit:
                # print('Failure')
                x[p, d] = time
                break
            time += time_step

print(np.unravel_index(np.argmax(x), x.shape))
print(np.amax(x))
# plt.plot(x)
# plt.show()
