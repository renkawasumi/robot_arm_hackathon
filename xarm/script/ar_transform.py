#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import (
  PoseStamped,
  Pose,
  Point,
  Quaternion,
)
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int32
import math


class ARTransform():
  def __init__(self):
    print("***__init__***")
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.alvarCB)
    self.waste_ides = [
    1, 2, 3, 4, 5, 6, 10, 
    13, 14, 15, 16, 17, 18,
    31, 32, 33, 34, 35, 36,
    37, 38, 39, 40, 41, 42,
    49, 50, 51, 52, 53, 54, 
    55,
    61
    ]
    self.shelf_id = 0
    self.listener = tf.TransformListener()
    self.ar_pub = rospy.Publisher('/ar_pose', Pose, queue_size=10)
    self.ar_pose = geometry_msgs.msg.Pose()

  def alvarCB(self, markers):
    # print("***alvarCB***")
    for m in markers.markers:
      marker_id = m.id
      marker_pose = m.pose.pose
      pos = marker_pose.position
      ori = marker_pose.orientation

      for waste_id in self.waste_ides:
        if waste_id == marker_id:
          print("waste_id", waste_id)
          (trans,rot) = self.listener.lookupTransform('/world', '/ar_marker_%s' % (waste_id), rospy.Time(0))
          self.ar_pose.position.x = trans[0]
          self.ar_pose.position.y = trans[1]
          self.ar_pose.position.z = trans[2]
          self.ar_pose.orientation.x = rot[0]
          self.ar_pose.orientation.y = rot[1]
          self.ar_pose.orientation.z = rot[2]
          self.ar_pose.orientation.w = rot[3]
          self.ar_pub.publish(self.ar_pose)  
          rpy = tf.transformations.euler_from_quaternion((self.ar_pose.orientation.x, self.ar_pose.orientation.y ,self.ar_pose.orientation.z, self.ar_pose.orientation.w))
          print(trans[0], trans[1], trans[2])
          print(math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2]))
          return
        
    print("None")
    self.ar_pose.position.x = None
    self.ar_pose.position.y = None
    self.ar_pose.position.z = None
    self.ar_pose.orientation.x = None
    self.ar_pose.orientation.y = None
    self.ar_pose.orientation.z = None
    self.ar_pose.orientation.w = None
    self.ar_pub.publish(self.ar_pose) 


if __name__ == '__main__':
    rospy.init_node('ar_transform') 
    ar_tf = ARTransform() 
    rospy.spin()
