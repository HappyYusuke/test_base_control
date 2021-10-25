#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------
# Title: base_control.pyのテストノード
# Author: Yusuke Kanazawa
# Date: 2020/10/20
# Memo:
#--------------------------------------------------
import rospy
import roslib
import sys

file_path = roslib.packages.get_pkg_dir("happymimi_teleop") + "/src/"
sys.path.insert(0, file_path)
from base_control import BaseControl


def main():
    bc = BaseControl()

    #bc.translateDist(0.1, 0.2)
    bc.rotateAngle(45)


if __name__ == "__main__":
    rospy.init_node("test_base")
    main()
