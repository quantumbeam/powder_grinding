#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32


def callback(data):
    # Extract the force.z value
    force_z = data.wrench.force.z
    # Publish the force.z value
    pub.publish(Float32(force_z))


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("force_z_publisher", anonymous=True)
    wrench_topic = rospy.get_param("~wrench_topic", "/wrench")

    # Create a publisher for publishing std_msgs/Float32 messages
    pub_topic = wrench_topic + "/force_z"
    pub = rospy.Publisher(pub_topic, Float32, queue_size=1)

    # Wait for the first message on the wrench_topic
    try:
        rospy.wait_for_message(wrench_topic, WrenchStamped, timeout=5)
    except rospy.ROSException:
        rospy.logerr(f"Timeout waiting for {wrench_topic}. Exiting.")
        exit()

    # Create a subscriber to the wrench_stamped topic
    rospy.Subscriber(wrench_topic, WrenchStamped, callback)

    # Keep the node running
    rospy.spin()
