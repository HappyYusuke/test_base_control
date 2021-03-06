#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class BaseControl():
    def __init__(self):
        # Publisher
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.odomCB)
        # Value
        self.twist_value = Twist()
        self.target_time = 0.0
        self.rate = rospy.Rate(1.0)
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
        if self.twist_value.angular.z > 0.0:
            while self.target_deg >= self.current_deg:
                self.twist_pub.publish(self.twist_value)
                if self.current_deg < -179:
                    while self.sub_target_deg >= self.current_deg:
                        self.twist_pub.publish(self.twist_value)
                    break
                else:
                    pass
        elif self.twist_value.angular.z < 0.0:
            while self.target_deg <= self.current_deg:
                self.twist_pub.publish(self.twist_value)
                if self.current_deg > 179:
                    while self.sub_target_deg <= self.current_deg:
                        self.twist_pub.publish(self.twist_value)
                    break
                else:
                    pass
        else:
            print("translateDist")
            start_time = time.time()
            end_time = time.time() + 0.15 # ????????????????????????0.15
            while end_time - start_time <= self.target_time:
                #print(end_time - start_time)
                self.twist_pub.publish(self.twist_value)
                end_time = time.time()
                self.rate.sleep()# ?????????????????????while?????????????????????????????????
        self.twist_value.linear.x = 0.0
        self.twist_value.angular.z = 0.0
        self.twist_pub.publish(self.twist_value)
        print("final deg: " + str(self.current_deg))

    def translateDist(self, dist, speed = 0.2):
        speed = abs(speed)
        self.target_time = abs(dist / speed)
        self.twist_value.linear.x = dist/abs(dist)*speed
        self.twist_value.angular.z = 0.0
        self.publishTwist()

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
