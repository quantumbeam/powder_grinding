#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import WrenchStamped, Point
from grinding_force_torque.srv import (
    WrenchStatistics,
    WrenchStatisticsResponse,
)

# Global variables
is_recording = False
forces_x = []
forces_y = []
forces_z = []
torques_x = []
torques_y = []
torques_z = []


def handle_start_stop(req):
    global is_recording, forces_x, forces_y, forces_z, torques_x, torques_y, torques_z
    print(f"Request: {req.command}")
    if req.command == "start":
        is_recording = True
        forces_x.clear()
        forces_y.clear()
        forces_z.clear()
        torques_x.clear()
        torques_y.clear()
        torques_z.clear()
        # Assuming WrenchStatisticsResponse is updated to include torque fields
        return WrenchStatisticsResponse(True, 0,0,0,0,0,0, 0,0,0,0,0,0) # Added placeholders for torque
    elif req.command == "stop":
        is_recording = False
        # Compute the averages and return
        (avg_fx, avg_fy, avg_fz, var_fx, var_fy, var_fz,
         avg_tx, avg_ty, avg_tz, var_tx, var_ty, var_tz) = compute_statistics()

        # Assuming WrenchStatisticsResponse is updated to include torque fields
        return WrenchStatisticsResponse(
            True, avg_fx, avg_fy, avg_fz, var_fx, var_fy, var_fz,
            avg_tx, avg_ty, avg_tz, var_tx, var_ty, var_tz
        )


def compute_statistics():
    global forces_x, forces_y, forces_z, torques_x, torques_y, torques_z
    avg_fx = np.average(forces_x) if forces_x else 0.0
    avg_fy = np.average(forces_y) if forces_y else 0.0
    avg_fz = np.average(forces_z) if forces_z else 0.0
    var_fx = np.var(forces_x) if forces_x else 0.0
    var_fy = np.var(forces_y) if forces_y else 0.0
    var_fz = np.var(forces_z) if forces_z else 0.0

    avg_tx = np.average(torques_x) if torques_x else 0.0
    avg_ty = np.average(torques_y) if torques_y else 0.0
    avg_tz = np.average(torques_z) if torques_z else 0.0
    var_tx = np.var(torques_x) if torques_x else 0.0
    var_ty = np.var(torques_y) if torques_y else 0.0
    var_tz = np.var(torques_z) if torques_z else 0.0

    rospy.loginfo(f"Average Force X: {avg_fx:.3f}, Y: {avg_fy:.3f}, Z: {avg_fz:.3f}")
    rospy.loginfo(f"Variance Force X: {var_fx:.3f}, Y: {var_fy:.3f}, Z: {var_fz:.3f}")
    rospy.loginfo(f"Average Torque X: {avg_tx:.3f}, Y: {avg_ty:.3f}, Z: {avg_tz:.3f}")
    rospy.loginfo(f"Variance Torque X: {var_tx:.3f}, Y: {var_ty:.3f}, Z: {var_tz:.3f}")

    return (avg_fx, avg_fy, avg_fz, var_fx, var_fy, var_fz,
            avg_tx, avg_ty, avg_tz, var_tx, var_ty, var_tz)


def wrench_callback(wrench_msg):
    global forces_x, forces_y, forces_z, torques_x, torques_y, torques_z, is_recording

    if is_recording:
        try:
            forces_x.append(wrench_msg.wrench.force.x)
            forces_y.append(wrench_msg.wrench.force.y)
            forces_z.append(wrench_msg.wrench.force.z)
            torques_x.append(wrench_msg.wrench.torque.x)
            torques_y.append(wrench_msg.wrench.torque.y)
            torques_z.append(wrench_msg.wrench.torque.z)

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.logwarn(f"TF error: {e}")
            return
        # print(f"Force : {wrench_msg.wrench.force}")


if __name__ == "__main__":
    rospy.init_node("wrench_statistics")

    # Create a TF listener
    tf_listener = tf.TransformListener()

    # Create a service server
    service_name=rospy.get_param("~wrench_statistics_service")
    service = rospy.Service(
        service_name, WrenchStatistics, handle_start_stop
    )

    # Create subscriber for WrenchStamped
    wrench_topic=rospy.get_param("~wrench_topic")
    wrench_sub = rospy.Subscriber(wrench_topic, WrenchStamped, wrench_callback)

    rospy.spin()
