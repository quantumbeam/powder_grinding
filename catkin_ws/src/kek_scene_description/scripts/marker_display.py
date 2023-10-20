import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class MarkerDisplay(object):

    last_index = 0

    def __init__(self, marker_publisher_name):
        self.publisher = rospy.Publisher(
            marker_publisher_name, MarkerArray, queue_size=10
        )
        self.marker_array = MarkerArray()
        self.marker = Marker()

    def display_waypoints(self, waypoints):
        pub = self.publisher
        marker = self.marker
        marker_array = self.marker_array
        rate = rospy.Rate(25)
        last_index = self.last_index
        # while not rospy.is_shutdown():
        for index, points in enumerate(waypoints):
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = last_index + index
            marker.action = Marker.ADD

            # marker.pose.position.x = 0.2
            # marker.pose.position.y = 0.2
            # marker.pose.position.z = 0.2
            marker.pose.position = points.position

            # marker.pose.orientation.x = 0.0
            # marker.pose.orientation.y = 0.0
            # marker.pose.orientation.z = 0.0
            # marker.pose.orientation.w = 0.0
            marker.pose.orientation = points.orientation

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            # marker.scale.x = 0.01
            # marker.scale.y = 0.01
            # marker.scale.z = 0.1

            marker.lifetime = rospy.Duration()  # existing marker for ever

            marker.type = marker.SPHERE
            # print(marker)
            marker_array.markers.append(marker)

            rate.sleep()  # sleepかませないとpublihが早すぎるのかうまくいかない

            pub.publish(marker_array)
            last_index += 1
        self.last_index = last_index

    def display_tf(self, tf_array):
        pub = self.publisher
        marker = self.marker
        marker_array = self.marker_array
        rate = rospy.Rate(100)
        last_index = self.last_index
        # while not rospy.is_shutdown():
        for index, tf in enumerate(tf_array):

            marker.header.frame_id = tf.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "tf"
            marker.id = last_index + index
            marker.action = Marker.ADD

            marker.pose.position = tf.transform.translation
            marker.pose.orientation = tf.transform.rotation

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01

            marker.type = marker.SPHERE
            marker.lifetime = rospy.Duration()  # existing marker for ever

            marker_array.markers.append(marker)

            rate.sleep()  # sleepかませないとpublihが早すぎるのかうまくいかない

            pub.publish(marker_array)
            last_index += 1
            if last_index > 20:
                last_index = 0
            print(last_index)
        self.last_index = last_index
