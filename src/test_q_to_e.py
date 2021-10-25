#!/usr/bin/env python
import rospy
import tf
from move_base_msgs.msg import MoveBaseGoal


def main():
    goal = MoveBaseGoal()

    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.7071
    goal.target_pose.pose.orientation.w = 0.7071
    
    quaternion_value = (
        goal.target_pose.pose.orientation.x,
        goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z,
        goal.target_pose.pose.orientation.w)

    print(quaternion_value)

    euler = tf.transformations.euler_from_quaternion(quaternion_value)

    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    print(str(roll) + ", " + str(pitch) + ", " + str(yaw))


if __name__ == "__main__":
    rospy.init_node("test_q_to_e")
    main()
