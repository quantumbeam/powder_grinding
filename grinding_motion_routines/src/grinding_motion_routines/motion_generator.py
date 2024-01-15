#!/usr/bin/env python3


# python-related imports
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp

# constants
from numpy import pi, nan


class MotionGenerator:
    def __init__(self, mortar_top_center_position, mortar_inner_scale):
        """Supported type : 'Dict' mortar_base_position [x:,y:,z:], 'Dict' mortar_inner_scale [x:,y:,z:]"""

        self.mortar_top_center_position = mortar_top_center_position
        self.mortar_inner_scale = mortar_inner_scale

    def _calc_quaternion_of_mortar_inner_wall(
        self, position, angle_param, yaw_bias, fixed_quaternion=False
    ):
        quats = []

        pos_x = np.array(position[0])
        pos_y = np.array(position[1])
        pos_z = np.array(position[2])

        #################### calculate orientation
        # angle param < 0  use inverse x,y, mean using inverse slope
        if angle_param < 0:
            pos_x *= -1
            pos_y *= -1

        # normalized position
        norm = np.sqrt(pos_x**2 + pos_y**2 + pos_z**2)
        normalized_pos_x = pos_x / norm
        normalized_pos_y = pos_y / norm
        normalized_pos_z = pos_z / norm

        # calc yaw angle
        yaw = np.arctan2(self.mortar_top_center_position["y"], self.mortar_top_center_position["x"]) + yaw_bias

        # rotate xy by the amount of yaw angle
        r, theta = self._cartesian_to_polar(normalized_pos_x, normalized_pos_y)
        rotated_normalized_pos_x, rotated_normalized_pos_y = self._polar_to_cartesian(
            r, theta + yaw
        )

        # calc euler of the normal and the verticle to the ground
        roll_of_the_normal = -np.arctan2(rotated_normalized_pos_y, normalized_pos_z)
        pitch_of_the_normal = -np.arctan2(
            rotated_normalized_pos_x,
            np.sqrt(rotated_normalized_pos_y**2 + normalized_pos_z**2),
        )
        yaw_of_the_normal = np.full_like(pos_z, yaw)

        roll_of_the_vertical = np.full_like(pos_x, pi)
        pitch_of_the_vertical = np.full_like(pos_y, 0)
        yaw_of_the_vertical = np.full_like(pos_z, yaw)

        for r_normal, p_normal, y_normal, r_vertical, p_vertical, y_vertical in zip(
            roll_of_the_normal,
            pitch_of_the_normal,
            yaw_of_the_normal,
            roll_of_the_vertical,
            pitch_of_the_vertical,
            yaw_of_the_vertical,
        ):
            rotation_normal = Rotation.from_euler("xyz", [r_normal, p_normal, y_normal])
            rotation_vertical = Rotation.from_euler(
                "xyz", [r_vertical, p_vertical, y_vertical]
            )
            rotations = Rotation.from_quat(
                [rotation_vertical.as_quat(), rotation_normal.as_quat()]
            )

            slerp = Slerp([0, 1], rotations)
            slerp_quat = slerp(angle_param).as_quat()
            quats.append(slerp_quat)

        quats = np.array(quats)

        if fixed_quaternion:
            quats = np.full_like(quats, quats[0])

        return quats

    def _ellipsoid_z_lower(self, x, y, radius):
        # x^2/rx^2+y^2/ry^2+z^2/rz^2=1より z = sqrt(rz^2(-x^2/rx^2-y^2/ry^2+1))
        rx, ry, rz = radius[0], radius[1], radius[2]

        buf = 1 - ((x**2) / (rx**2)) - ((y**2) / (ry**2))
        z = -np.sqrt(rz**2 * buf)  # 楕円体の下半分(lower)なのでマイナスつける
        return z

    def _cartesian_to_polar(self, x, y):  # retern 0 <= θ < 2pi
        theta = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)

        return r, theta

    def _polar_to_cartesian(self, r, theta):
        x = r * np.cos(theta)
        y = r * np.sin(theta)

        return x, y

    def _lerp_in_cartesian(self, st, ed, points):
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

    def _lerp_in_polar(self, st, ed, points, number_of_rotations, r_max):
        st_r, st_theta = self._cartesian_to_polar(st[0], st[1])
        ed_r, ed_theta = self._cartesian_to_polar(ed[0], ed[1])

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

    def update_mortar_position(self, pos):
        self.mortar_top_center_position = pos

    def create_circular_waypoints(
        self,
        begining_position,
        end_position,
        begining_radious_z,
        end_radious_z,
        angle_param=0,
        yaw_bias=0,
        number_of_rotations=1,
        number_of_waypoints_per_circle=10,
        center_position=np.array([0, 0]),
    ):
        """
        supported type
        begining_position : list [x,y]
        end_position : list [x,y]
        begining_radious_z : float
        end_radious_z : float
        angle_param : float
        yaw_bias : float
        number_of_rotations : int
        number_of_waypoints_per_circle : int
        center_position : list [x,y]
        """
        # chnage unit from mm to m
        begining_position = np.array(begining_position).astype(np.float64) * 0.001
        end_position = np.array(end_position).astype(np.float64) * 0.001
        begining_radious_z = float(begining_radious_z) * 0.001
        end_radious_z = float(end_radious_z) * 0.001
        circular_center_position = np.array(center_position).astype(np.float64) * 0.001
        total_number_of_waypoints = number_of_rotations * number_of_waypoints_per_circle

        if number_of_rotations < 1:
            raise ValueError(
                "Can't define end θ, you can choose number_of_rotations >= 1"
            )
        elif number_of_waypoints_per_circle < 1:
            raise ValueError(
                "Can't calculate motion, you can choose number_of_waypoints_per_circle >= 1"
            )
        #################### calculate position
        # calc xy
        x, y = self._lerp_in_polar(
            begining_position,
            end_position,
            total_number_of_waypoints,
            number_of_rotations,
            self.mortar_inner_scale["x"],
        )

        # shift center pos
        x += circular_center_position[0]
        y += circular_center_position[1]

        # check xy in range
        if np.any(np.abs(x) > self.mortar_inner_scale["x"]):
            raise ValueError("calculated x is over mortar scale")

        elif np.any(np.abs(y) > self.mortar_inner_scale["y"]):
            raise ValueError("calculated y is over mortar scale")

        # calc z
        if end_radious_z < begining_radious_z:
            raise ValueError(
                "Calc error: begining radius z is bigger than end radius z."
            )
            return False
        radious_z = np.linspace(
            begining_radious_z, end_radious_z, total_number_of_waypoints, endpoint=False
        )
        z = self._ellipsoid_z_lower(
            x,
            y,
            [self.mortar_inner_scale["x"], self.mortar_inner_scale["y"], radious_z],
        )

        position = np.array([x, y, z])

        # shift to work pos
        shifted_position = np.array(
            [
                position[0] + self.mortar_top_center_position["x"],
                position[1] + self.mortar_top_center_position["y"],
                position[2] + self.mortar_top_center_position["z"],
            ]
        )

        #################### calculate orientation
        if angle_param == 0:
            x_for_quat = np.full_like(x, circular_center_position[0])
            y_for_quat = np.full_like(y, circular_center_position[1])
            pos_for_quat = np.array([x_for_quat, y_for_quat, z])
            quat = self._calc_quaternion_of_mortar_inner_wall(
                pos_for_quat, 1.0, yaw_bias
            )
        else:
            quat = self._calc_quaternion_of_mortar_inner_wall(
                position, angle_param, yaw_bias
            )

        #################### create waypoints
        waypoints = np.stack(
            [
                shifted_position[0],
                shifted_position[1],
                shifted_position[2],
                quat.T[0],
                quat.T[1],
                quat.T[2],
                quat.T[3],
            ]
        ).T
        # delete duplicated waypoints
        # waypoints, index = np.unique(waypoints, axis=0, return_index=True)
        # waypoints = waypoints[np.argsort(index)]

        return waypoints

    def create_cartesian_waypoints(
        self,
        begining_position,
        end_position,
        begining_radius_z,
        end_radius_z,
        angle_param=0,
        fixed_quaternion=False,
        yaw_bias=0,
        number_of_waypoints=5,
    ):
        """
        supported type
        begining_theta : float
        end_tehta : float
        begining_length_from_center : float
        end_length_from_center : float
        begining_radius_z : float
        end_radius_z : float
        angle_param : float
        fixed_quaternion : bool
        yaw_bias : float
        number_of_waypoints : int
        motion_counts : int
        """
        if number_of_waypoints < 1:
            raise ValueError(
                "Can't calculate motion, you can choose number_of_waypoints >= 1"
            )

        # chnage unit from mm to m
        begining_position = np.array(begining_position).astype(np.float64) * 0.001
        end_position = np.array(end_position).astype(np.float64) * 0.001
        begining_radius_z *= 0.001
        end_radius_z *= 0.001

        # calculate position
        x, y = self._lerp_in_cartesian(
            begining_position, end_position, number_of_waypoints
        )
        radius_z = np.linspace(
            begining_radius_z, end_radius_z, number_of_waypoints, endpoint=False
        )
        z = self._ellipsoid_z_lower(x, y, radius_z)
        position = np.array([x, y, z])

        # shift to work pos
        shifted_position = np.array(
            [
                position[0] + self.mortar_top_center_position["x"],
                position[1] + self.mortar_top_center_position["y"],
                position[2] + self.mortar_top_center_position["z"],
            ]
        )

        #################### calculate orientation
        quat = self._calc_quaternion_of_mortar_inner_wall(
            position=position,
            angle_param=angle_param,
            yaw_bias=yaw_bias,
            fixed_quaternion=fixed_quaternion,
        )

        #################### create waypoints
        waypoints = np.stack(
            [
                shifted_position[0],
                shifted_position[1],
                shifted_position[2],
                quat.T[0],
                quat.T[1],
                quat.T[2],
                quat.T[3],
            ]
        ).T
        # delete duplicated waypoints
        waypoints, index = np.unique(waypoints, axis=0, return_index=True)
        waypoints = waypoints[np.argsort(index)]

        return waypoints

    def create_liner_waypoints_list(
        self,
        begining_theta,
        end_tehta,
        begining_length_from_center,
        end_length_from_center,
        begining_radius_z,
        end_radius_z,
        angle_param=0,
        fixed_quaternion=False,
        yaw_bias=0,
        number_of_waypoints=5,
        motion_counts=1,
    ):
        """
        supported type
        begining_theta : float
        end_tehta : float
        begining_length_from_center : float
        end_length_from_center : float
        begining_radius_z : float
        end_radius_z : float
        angle_param : float
        fixed_quaternion : bool
        yaw_bias : float
        number_of_waypoints : int
        motion_counts : int
        """
        if number_of_waypoints < 1:
            raise ValueError(
                "Can't calculate motion, you can choose number_of_waypoints >= 1"
            )

        # chnage unit from mm to m
        begining_length_from_center *= 0.001
        end_length_from_center *= 0.001
        begining_radius_z *= 0.001
        end_radius_z *= 0.001

        # calculate position in tahta range
        theta = np.linspace(begining_theta, end_tehta, motion_counts, endpoint=False)
        begining_r = np.full_like(theta, begining_length_from_center)
        end_r = np.full_like(theta, end_length_from_center)
        begining_x, begining_y = self._polar_to_cartesian(begining_r, theta)
        end_x, end_y = self._polar_to_cartesian(end_r, theta)
        radius_z = np.linspace(
            begining_radius_z, end_radius_z, number_of_waypoints, endpoint=False
        )

        waypoints_list = []
        for begining_x, begining_y, end_x, end_y in zip(
            begining_x, begining_y, end_x, end_y
        ):
            #################### calculate position
            begining_position = [begining_x, begining_y]
            end_position = [end_x, end_y]
            x, y = self._lerp_in_cartesian(
                begining_position, end_position, number_of_waypoints
            )
            z = self._ellipsoid_z_lower(x, y, radius_z)
            position = np.array([x, y, z])

            # shift to work pos
            shifted_position = np.array(
                [
                    position[0] + self.mortar_top_center_position["x"],
                    position[1] + self.mortar_top_center_position["y"],
                    position[2] + self.mortar_top_center_position["z"],
                ]
            )

            #################### calculate orientation
            quat = self._calc_quaternion_of_mortar_inner_wall(
                position=position,
                angle_param=angle_param,
                yaw=yaw_bias,
                fixed_quaternion=fixed_quaternion,
            )

            #################### create waypoints
            waypoints = np.stack(
                [
                    shifted_position[0],
                    shifted_position[1],
                    shifted_position[2],
                    quat.T[0],
                    quat.T[1],
                    quat.T[2],
                    quat.T[3],
                ]
            ).T
            # delete duplicated waypoints
            waypoints, index = np.unique(waypoints, axis=0, return_index=True)
            waypoints = waypoints[np.argsort(index)]

            #################### append list
            waypoints_list.append(waypoints)

        return waypoints_list
