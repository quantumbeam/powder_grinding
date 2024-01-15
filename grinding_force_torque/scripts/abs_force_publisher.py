#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64

import time
from math import sqrt


class AbsoluteForce:
    def __init__(self):
        wrench_topic = rospy.get_param("~wrench_topic", "/wrench")
        self.sub = rospy.Subscriber(wrench_topic, WrenchStamped, self.callback)
        # Publisherの作成
        self.pub = rospy.Publisher(wrench_topic+"/abs_force", Float64, queue_size=1)

        self.var_reset()
        self.publish_data_counts = 100

    def var_reset(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.counter = 0

    def callback(self, data):
        # forceの絶対値を送信
        self.x += data.wrench.force.x
        self.y += data.wrench.force.y
        self.z += data.wrench.force.z

        self.counter += 1
        if (self.counter % self.publish_data_counts) == 0:
            x = self.x / self.publish_data_counts
            y = self.y / self.publish_data_counts
            z = self.z / self.publish_data_counts
            self.publish(sqrt(x**2 + y**2 + z**2))
            self.var_reset()

    def publish(self, data):
        self.pub.publish(data)

    def function(self, data):
        # そのほかの処理もあったら書く

        return data


if __name__ == "__main__":
    rospy.init_node("abs_force_node")
    node = AbsoluteForce()
    rate = rospy.Rate(100)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
