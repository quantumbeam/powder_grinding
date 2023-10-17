#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospy

from numpy import pi


def ellipsoid_z_lower(x, y, radius):
    # x^2/rx^2+y^2/ry^2+z^2/rz^2=1より z = sqrt(rz^2(-x^2/rx^2-y^2/ry^2+1))
    rx, ry, rz = radius[0], radius[1], radius[2]

    buf = 1 - ((x**2) / (rx**2)) - ((y**2) / (ry**2))
    z = -np.sqrt(rz**2 * buf)  # 楕円体の下半分(lower)なのでマイナスつける
    return z


def cartesian_to_polar(x, y):  # retern 0 <= θ < 2pi
    theta = np.arctan2(y, x)
    r = np.sqrt(x**2 + y**2)

    return r, theta


def polar_to_cartesian(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    return x, y


def lerp_in_cartesian(st, ed, points):
    st_x = st[0]
    st_y = st[1]
    ed_x = ed[0]
    ed_y = ed[1]
    dx = abs(ed_x - st_x)
    dy = abs(ed_y - st_y)
    if dx > dy:
        x = np.linspace(st_x, ed_x, points, endpoint=False)
        y = st_y + (ed_y - st_y) * (x - st_x) / (ed_x - st_x)

    else:
        y = np.linspace(st_y, ed_y, points, endpoint=False)
        x = st_x + (ed_x - st_x) * (y - st_y) / (ed_y - st_y)

    return x, y


def lerp_in_polar(st, ed, points, number_of_rotations, r_max):
    if number_of_rotations < 1:
        rospy.loginfo("Can't define end θ, you can choose number_of_rotations >= 1")

    st_r, st_theta = cartesian_to_polar(st[0], st[1])
    ed_r, ed_theta = cartesian_to_polar(ed[0], ed[1])

    if st_theta > ed_theta:
        ed_theta += 2 * pi
    ed_theta += (number_of_rotations - 1) * 2 * pi

    dr = abs(ed_r - st_r) / (r_max * 2)
    d_theta = abs(ed_theta - st_theta) / (2 * pi)

    if dr > d_theta:
        r = np.linspace(st_r, ed_r, points, endpoint=False)
        theta = st_theta + (ed_theta - st_theta) * (r - st_r) / (ed_r - st_r)
    else:
        theta = np.linspace(st_theta, ed_theta, points, endpoint=False)
        r = st_r + (ed_r - st_r) * (theta - st_theta) / (ed_theta - st_theta)
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    return x, y


def plot_position_to_debug(position, show=True):
    position *= 1000  # cahnge unit m to mm
    x, y, z = position[0], position[1], position[2]

    deb_fig2d = plt.figure()
    deb_ax2d = deb_fig2d.add_subplot(111)
    deb_fig3d = plt.figure()
    deb_ax3d = Axes3D(deb_fig3d)

    # waypoints
    deb_ax2d.scatter(x, y)
    if show:
        # mortar
        # deb_ax2d.axhline(0, linewidth=2, color="gray")  # 横軸(horizon)ゼロの太線化
        # deb_ax2d.axvline(0, linewidth=2, color="gray")  # 縦軸(Vertical)ゼロの太線化
        deb_ax2d.set_aspect("equal")
        plt.show()

    # debug 3D plot
    deb_ax3d.scatter(x, y, z)  # yxzの座標系なのに注意
    if show:
        deb_ax3d.set_xlabel("X")
        deb_ax3d.set_ylabel("Y")
        deb_ax3d.set_zlabel("Z")
        plt.show()
