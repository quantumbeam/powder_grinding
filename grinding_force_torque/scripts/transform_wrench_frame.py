#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix
import numpy as np

class ForceTorqueConverter:
    def __init__(self):
        rospy.init_node('force_torque_converter', anonymous=True)

        # Load parameters from the parameter server
        self.input_topic = rospy.get_param("~input_topic", "/wrench") 
        self.output_topic = rospy.get_param("~output_topic", "/wrench_on_pestle_tip") 
        self.ft_sensor_frame = rospy.get_param("~ft_sensor_frame", "tool0")  
        self.target_frame = rospy.get_param("~target_frame", "pestle_tip")  
        self.invert_wrench = rospy.get_param("~invert_wrench", False)

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscriber and Publisher
        self.ft_sub = rospy.Subscriber(self.input_topic, geometry_msgs.msg.WrenchStamped, self.ft_callback)
        self.ft_pub = rospy.Publisher(self.output_topic, geometry_msgs.msg.WrenchStamped, queue_size=10)

        rospy.loginfo(f"Force/Torque Converter initialized:")
        rospy.loginfo(f"  Input Topic: {self.input_topic}")
        rospy.loginfo(f"  FT Sensor Frame: {self.ft_sensor_frame}")
        rospy.loginfo(f"  Target Frame: {self.target_frame}")
        rospy.loginfo(f"  Invert Wrench: {self.invert_wrench}")
        rospy.loginfo(f"  Output Topic: {self.output_topic}")

    def ft_callback(self, msg):
        try:
            # Get the transform from the sensor frame to the base frame
            transform = self.tf_buffer.lookup_transform(
                self.ft_sensor_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
            )

            # Convert the transform to a homogeneous matrix
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            rotation = np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            rotation_matrix = quaternion_matrix(rotation)[:3, :3]
            homogeneous_matrix = np.eye(4)
            homogeneous_matrix[:3, :3] = rotation_matrix
            homogeneous_matrix[:3, 3] = translation

            # Get the transform from the target frame to the base frame
            transform_target = self.tf_buffer.lookup_transform(
                self.ft_sensor_frame, self.target_frame, rospy.Time(0), rospy.Duration(1.0)
            )

            # Convert the target transform to a homogeneous matrix
            translation_target = np.array([
                transform_target.transform.translation.x,
                transform_target.transform.translation.y,
                transform_target.transform.translation.z
            ])
            rotation_target = np.array([
                transform_target.transform.rotation.x,
                transform_target.transform.rotation.y,
                transform_target.transform.rotation.z,
                transform_target.transform.rotation.w
            ])
            rotation_matrix_target = quaternion_matrix(rotation_target)[:3, :3]
            homogeneous_matrix_target = np.eye(4)
            homogeneous_matrix_target[:3, :3] = rotation_matrix_target
            homogeneous_matrix_target[:3, 3] = translation_target

            # Calculate the relative transform from sensor to target
            relative_transform_matrix = np.linalg.inv(homogeneous_matrix_target) @ homogeneous_matrix

            # Extract the relative rotation and translation
            relative_rotation = relative_transform_matrix[:3, :3]
            relative_translation = relative_transform_matrix[:3, 3]

            # Convert the force and torque to numpy arrays
            force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

            # Invert force and torque if the parameter is set
            if self.invert_wrench:
                rospy.logdebug_throttle(1.0, "Inverting wrench due to 'invert_wrench' parameter.")
                force = -force
                torque = -torque
            else:
                rospy.logdebug_throttle(1.0, "Not inverting wrench.")


            # Transform the force to the target frame
            transformed_force = relative_rotation @ force

            # Transform the torque to the target frame and add the cross product
            transformed_torque = relative_rotation @ torque + np.cross(relative_translation, transformed_force)

            # Create a new WrenchStamped message
            output_msg = geometry_msgs.msg.WrenchStamped()
            output_msg.header.stamp = msg.header.stamp
            output_msg.header.frame_id = self.target_frame
            output_msg.wrench.force.x = transformed_force[0]
            output_msg.wrench.force.y = transformed_force[1]
            output_msg.wrench.force.z = transformed_force[2]
            output_msg.wrench.torque.x = transformed_torque[0]
            output_msg.wrench.torque.y = transformed_torque[1]
            output_msg.wrench.torque.z = transformed_torque[2]

            # Publish the transformed wrench
            self.ft_pub.publish(output_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF error: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = ForceTorqueConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
