#!/usr/bin/env python
# -*- coding: utf-8 -*=
import rospy
import math
import tf
from nav_msgs.msg import Odometry


class QuaternionToEuler():
    def __init__(self):
        # Subscriber
        rospy.Subscriber('/vmegarover/diff_drive_controller/odom', Odometry, self.odomCB)
        # Value
        self.quaternion = (0.0, 0.0, 0.0, 0.0)
        self.current_euler = []
        self.current_deg = 0.0
        self.target_deg = 0.0

    def odomCB(self, receive_msg):
        self.quaternion = (
            receive_msg.pose.pose.orientation.x,
            receive_msg.pose.pose.orientation.y,
            receive_msg.pose.pose.orientation.z,
            receive_msg.pose.pose.orientation.w)
        self.current_euler = tf.transformations.euler_from_quaternion(self.quaternion)
        self.current_deg = math.degrees(self.current_euler[2])

    def execute(self):
        while not rospy.is_shutdown():
            print(self.current_deg)




if __name__ == '__main__':
    rospy.init_node("exe_QtoE")
    qte = QuaternionToEuler()
    qte.execute()
