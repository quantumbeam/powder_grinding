#!/usr/bin/env python3

import rospy
import tf
import numpy as np
from geometry_msgs.msg import WrenchStamped, Point
from grinding_motion_routines.srv import (
    PositionCalibrateVector,
    PositionCalibrateVectorResponse,
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
        return PositionCalibrateVectorResponse(True, 0, 0, 0, 0, 0, 0)
    elif req.command == "stop":
        is_recording = False
        # Compute the integrals and return
        integral_x, integral_y, integral_z, var_x, var_y, var_z = compute_statistics()

        return PositionCalibrateVectorResponse(
            True, integral_x, integral_y, integral_z, var_x, var_y, var_z
        )


def compute_statistics():
    global forces_x, forces_y
    integral_x = sum(forces_x)
    integral_y = sum(forces_y)
    integral_z = sum(forces_z)
    var_x = np.var(forces_x)
    var_y = np.var(forces_y)
    var_z = np.var(forces_z)
    rospy.loginfo(f"Integral x: {integral_x}")
    rospy.loginfo(f"Integral y: {integral_y}")
    rospy.loginfo(f"Integral z: {integral_z}")
    rospy.loginfo(f"Variance x: {var_x}")
    rospy.loginfo(f"Variance y: {var_y}")
    rospy.loginfo(f"Variance z: {var_z}")

    return integral_x, integral_y, integral_z, var_x, var_y, var_z


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
        print(f"Force : {wrench_msg.wrench.force}")


if __name__ == "__main__":
    rospy.init_node("wrench_statistics")

    # Create a TF listener
    tf_listener = tf.TransformListener()

    # Create a service server
    service_name=rospy.get_param("~wrench_statistics_service")
    service = rospy.Service(
        service_name, PositionCalibrateVector, handle_start_stop
    )

    # Create subscriber for WrenchStamped
    wrench_topic=rospy.get_param("~wrench_topic")
    wrench_sub = rospy.Subscriber(wrench_topic, WrenchStamped, wrench_callback)

    rospy.spin()