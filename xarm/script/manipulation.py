#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import tf
import rospkg
from geometry_msgs.msg import (
  PoseStamped,
  Pose,
  Point,
  Quaternion,
)

class Manipulation(object):
    def __init__(self):
        print("***Manipulation_initial***")
        super(Manipulation, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface() 
        self.group = moveit_commander.MoveGroupCommander("xarm6")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
        self.target_pose = geometry_msgs.msg.Pose()
        self.ar_pose = geometry_msgs.msg.Pose()

    def plan(self):
        print("***generate_plan***") 
        self.group.set_max_velocity_scaling_factor(1.0)
        self.group.set_pose_target(self.target_pose)
        plan = self.group.plan()    
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(1)
        return plan 
  
    def execute(self):
        print("***execute***")
        self.group.go(wait=True)
        self.group.clear_pose_targets()
  
    def plan_and_execute(self):
        self.plan()
        self.execute()

    def stop(self):
        print("***stop***")
        self.group.stop() 

    def set_target_pose_quaternion(self, x, y, z, qx, qy, qz, qw):
        print("***set_pose_quaternion***")
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.position.z = z
        self.target_pose.orientation.x = qx
        self.target_pose.orientation.y = qy
        self.target_pose.orientation.z = qz
        self.target_pose.orientation.w = qw
 
    def set_target_pose_euler(self, x, y, z, roll, pitch, yaw):
        print("***set_pose_euler***")
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.position.z = z
        self.target_pose.orientation.x = q[0]
        self.target_pose.orientation.y = q[1]
        self.target_pose.orientation.z = q[2]
        self.target_pose.orientation.w = q[3]
   
if __name__=='__main__':
    try:
        rospy.init_node("xArm6_Moveit")
        xArm6()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
