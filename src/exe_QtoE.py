#!/usr/bin/env python
# -*- coding: utf-8 -*=
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class QuaternionToEuler():
    def __init__(self):
        # Publisher
        self.twist_pub = rospy.Publisher('/vmegarover/diff_drive_controller/cmd_vel', Twist, queue_size = 1)
        # Subscriber
        rospy.Subscriber('/vmegarover/diff_drive_controller/odom', Odometry, self.odomCB)
        # Value
        self.twist_value = Twist()
        self.quaternion = (0.0, 0.0, 0.0, 0.0)
        self.current_euler = []
        self.current_deg = 0.0
        self.target_deg = 0.0
        self.remain_deg = 0.0
        self.sub_target_deg = 0.0

    def odomCB(self, receive_msg):
        self.quaternion = (
            receive_msg.pose.pose.orientation.x,
            receive_msg.pose.pose.orientation.y,
            receive_msg.pose.pose.orientation.z,
            receive_msg.pose.pose.orientation.w)
        self.current_euler = tf.transformations.euler_from_quaternion(self.quaternion)
        self.current_deg = math.degrees(self.current_euler[2])

    def publishTwist(self):
        if self.twist_value.angular.z >= 0.0:
            while self.target_deg >= self.current_deg:
                self.twist_pub.publish(self.twist_value)
                if self.current_deg < -179:
                    while self.sub_target_deg >= self.current_deg:
                        self.twist_pub.publish(self.twist_value)
                    break
                else:
                    pass
        else:
            while self.target_deg <= self.current_deg:
                self.twist_pub.publish(self.twist_value)
                if self.current_deg > 179:
                    while self.sub_target_deg <= self.current_deg:
                        self.twist_pub.publish(self.twist_value)
                    break
                else:
                    pass
        self.twist_value.angular.z = 0.0
        self.twist_pub.publish(self.twist_value)
        print("final deg: " + str(self.current_deg))


    def rotateAngle(self, deg, speed = 0.5):
        rospy.sleep(0.05)
        if deg >= 0.0:
            self.target_deg = self.current_deg + deg
            if self.target_deg >= 180:
                self.remain_deg = self.target_deg - 180
                self.sub_target_deg = -180 + self.remain_deg
            else:
                pass
            self.twist_value.angular.z = speed
        else:
            self.target_deg = self.current_deg + deg
            if self.target_deg <= -180:
                self.remain_deg = self.target_deg + 180
                self.sub_target_deg = 180 + self.remain_deg
            else:
                pass
            self.twist_value.angular.z = -speed
        print("current deg: " + str(self.current_deg))
        print("target deg: " + str(self.target_deg))
        print("sub_target deg: " + str(self.sub_target_deg)) 

        self.publishTwist()


if __name__ == '__main__':
    rospy.init_node("exe_QtoE")
    qte = QuaternionToEuler()
    qte.rotateAngle(10.0)
