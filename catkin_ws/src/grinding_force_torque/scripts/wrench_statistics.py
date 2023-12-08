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


def handle_start_stop(req):
    global is_recording, forces_x, forces_y, forces_z
    print(f"Request: {req.command}")
    if req.command == "start":
        is_recording = True
        forces_x.clear()
        forces_y.clear()
        forces_z.clear()
        return WrenchStatisticsResponse(True, 0, 0, 0, 0, 0, 0)
    elif req.command == "stop":
        is_recording = False
        # Compute the averages and return
        average_x, average_y, average_z, var_x, var_y, var_z = compute_statistics()

        return WrenchStatisticsResponse(
            True, average_x, average_y, average_z, var_x, var_y, var_z
        )


def compute_statistics():
    global forces_x, forces_y
    average_x = np.average(forces_x)
    average_y = np.average(forces_y)
    average_z = np.average(forces_z)
    var_x = np.var(forces_x)
    var_y = np.var(forces_y)
    var_z = np.var(forces_z)
    rospy.loginfo(f"average x: {average_x}")
    rospy.loginfo(f"average y: {average_y}")
    rospy.loginfo(f"average z: {average_z}")
    rospy.loginfo(f"Variance x: {var_x}")
    rospy.loginfo(f"Variance y: {var_y}")
    rospy.loginfo(f"Variance z: {var_z}")

    return average_x, average_y, average_z, var_x, var_y, var_z


def wrench_callback(wrench_msg):
    global  forces_x, forces_y, forces_z, is_recording

    if is_recording:
        try:
            forces_x.append(wrench_msg.wrench.force.x)
            forces_y.append(wrench_msg.wrench.force.y)
            forces_z.append(wrench_msg.wrench.force.z)

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
