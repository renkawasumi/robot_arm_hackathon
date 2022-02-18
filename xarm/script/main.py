#!/usr/bin/env python
# coding: utf-8

import rospy
import geometry_msgs.msg
import tf
import math
import actionlib
from geometry_msgs.msg import Pose
# Arm
from xarm.msg import MoveAction
# Gripper
from xarm_gripper.msg import GripperAction
# Vision
from ar_track_alvar_msgs.msg import AlvarMarkers
# Voice
from gtts import gTTS
from mutagen.mp3 import MP3 as mp3
import pygame
import time


class Arm():
    def __init__(self):
        self.move_act = actionlib.SimpleActionClient('xarm6_moveit', MoveAction) 
        self.move_goal = MoveAction().action_goal.goal

    def init_pose(self):
        self.move_act.wait_for_server(rospy.Duration(10)) 
        self.move_goal.mode = True
        self.move_act.send_goal(self.move_goal, feedback_cb = self.plan_cb)
        move_result = self.move_act.wait_for_result(rospy.Duration(10))
        return move_result
       
    def move(self, target_pose):
        self.move_act.wait_for_server(rospy.Duration(10)) 
        self.move_goal.mode = False
        self.move_goal.target_pose = target_pose
        self.move_act.send_goal(self.move_goal, feedback_cb = self.plan_cb)
        move_result = self.move_act.wait_for_result(rospy.Duration(10))
        return move_result

    def stop(self):
        self.move_act.cancel_goal()

    def plan_cb(self, msg):
        print("plan_feedback", msg.plan)


class Gripper():
    def __init__(self):
        self.gripper_act = actionlib.SimpleActionClient('xarm/gripper_move', GripperAction)
        self.gripper_goal = GripperAction().action_goal.goal
        self.speed = 5000

    def set_speed(self, speed):
        self.speed = speed
   
    def open(self):
        self.gripper_act.wait_for_server(rospy.Duration(10))
        pos = 850
        self.gripper_goal.target_pulse = pos
        self.gripper_goal.pulse_speed = self.speed
        self.gripper_act.send_goal(self.gripper_goal, feedback_cb = self.gripper_cb)
        gripper_result = self.gripper_act.wait_for_result(rospy.Duration(3))
        return gripper_result
 
    def close(self): 
        self.gripper_act.wait_for_server(rospy.Duration(10))
        pos = 0
        self.gripper_goal.target_pulse = pos
        self.gripper_goal.pulse_speed = self.speed
        self.gripper_act.send_goal(self.gripper_goal, feedback_cb = self.gripper_cb)
        gripper_result = self.gripper_act.wait_for_result(rospy.Duration(3))
        return gripper_result

    def stop(self):
        self.gripper_act.cancel_goal()
 
    def gripper_cb(self, msg):
        print("gripper_feedback", msg.current_pulse)


class Vision():
    def __init__(self):
        rospy.Subscriber("/ar_pose", Pose, self.target_pose_cb)
        self.target_pose = geometry_msgs.msg.Pose()
    
    def target_pose_cb(self, msg):
        self.target_pose.position.x = msg.position.x
        self.target_pose.position.y = msg.position.y
        self.target_pose.position.z = msg.position.z 
        self.target_pose.orientation.x = msg.orientation.x 
        self.target_pose.orientation.y = msg.orientation.y
        self.target_pose.orientation.z = msg.orientation.z
        self.target_pose.orientation.w = msg.orientation.w


class Voice():
    def __init__(self):
        self.filename = "out.mp3"
        pygame.mixer.init()
    
    def text_to_mp3(self, text):
        tts = gTTS(text, lang="en", slow=True)
        tts.save(self.filename)

    def play(self):
        pygame.mixer.music.load(self.filename)
        mp3_length = mp3(self.filename).info.length
        pygame.mixer.music.play(1)
        time.sleep(mp3_length)
        pygame.mixer.music.stop() 

    def text_play(self, text):
        self.text_to_mp3(text)
        self.play()

def main():
    print("start main.py")
    rospy.init_node("main", anonymous=True) 
    arm = Arm()
    gripper = Gripper()
    vision = Vision()
    voice = Voice()

    voice.text_play("Initialize")
    result = arm.init_pose()
    gripper.set_speed(5000)
    result = gripper.open()

    voice.text_play("Looking for AR marker")
    arm.move(vision.target_pose)
    gripper.close()
    gripper.open()
    arm.init_pose()

    
      
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
