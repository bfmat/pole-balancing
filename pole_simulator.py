#!/usr/bin/env python3

import sys

import control
import matplotlib.pyplot as plt
import numpy as np

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor, QPen, QPalette, QFont

gravity = 9.81
cart_mass = 1
pole_mass = 0.1
pole_length = 0.5
track_limit = 10
failure_angle = 1  # radians
time_step = 0.001


def get_angle_accel(angle, angle_speed, force):
    return (
        ((gravity * np.sin(angle)) + (np.cos(angle) * (
            (-force - (pole_mass * pole_length * (angle_speed ** 2) * np.sin(angle))) / (cart_mass + pole_mass)
        )))
        / (pole_length * ((4/3) - ((pole_mass * (np.cos(angle) ** 2)) / (cart_mass + pole_mass))))
    )


def get_pos_accel(angle, angle_speed, angle_accel, force):
    return (
        (force + (pole_mass * pole_length * (((angle_speed ** 2) * np.sin(angle)) + (angle_accel * np.cos(angle)))))
        / (cart_mass + pole_mass)
    )


# State space control matrices (linearized)
A = np.array([[1, 1, 0, 0], [gravity / pole_length, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
B = np.array([[0], [-1], [0], [1]])
Q = 0.1 * np.diag([1 / (0.8 ** 2), 1 / (0.005 ** 2), 1 / (20 ** 2), 1 / (1 ** 2)])
R = np.array([[1 / (10 ** 2)]])
# Calculate LQR optimal control policy
K, _, _ = control.lqr(A, B, Q, R)

# Switches for output modes
log = False
graph = False
gui = True

time_steps = int(20 / time_step)
angle = 0
angle_speed = 0
pos = 9
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

if graph:
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


class PoleSimulatorVisualizer(QWidget):

    WIDTH = 2560
    HEIGHT = 200
    CART_POS_SCALE = WIDTH / (track_limit * 2)

    def __init__(self):
        super(PoleSimulatorVisualizer, self).__init__()
        self.setFixedSize(self.WIDTH, self.HEIGHT)
        self.show()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(time_step * 1000)
        self.data_iterator = zip(positions, angles)
        self.cart_pos = 0
        self.pole_angle = 0

    def update(self):
        try:
            self.cart_pos, self.pole_angle = next(self.data_iterator)
        except StopIteration:
            QApplication.quit()
        self.repaint()

    def paintEvent(self, _):
        painter = QPainter()
        painter.begin(self)
        pen = QPen(Qt.black, 8)
        painter.setPen(pen)
        painter.drawLine(0, self.HEIGHT - 20, self.WIDTH, self.HEIGHT - 20)
        pixel_pos = self.cart_pos * self.CART_POS_SCALE + (self.WIDTH // 2)
        painter.drawRect(pixel_pos - 20, self.HEIGHT - (40 + 20), 40, 40)
        pole_end_x = (np.sin(self.pole_angle) * pole_length * self.CART_POS_SCALE * 2) + pixel_pos
        pole_end_y = (np.cos(self.pole_angle) * pole_length * self.CART_POS_SCALE * -2) + (self.HEIGHT - (20 + 30))
        pen = QPen(Qt.red, 8)
        painter.setPen(pen)
        painter.drawLine(pixel_pos, self.HEIGHT - (20 + 30), pole_end_x, pole_end_y)
        painter.end()


if gui:
    app = QApplication([])
    ic = PoleSimulatorVisualizer()
    sys.exit(app.exec_())
