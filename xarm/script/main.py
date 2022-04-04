#!/usr/bin/env python
# coding: utf-8

import rospy
import geometry_msgs.msg
import tf
import math
import actionlib
from geometry_msgs.msg import Pose
import manipulation

def main():
    print("start main.py")
    rospy.init_node("main", anonymous=True) 
    mani = manipulation.Manipulation()
    mani.set_target_pose_euler(0.2, 0.0, 0.5, 0 + math.pi, 0, 0)
    mani.plan()
    mani.execute()
          
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
