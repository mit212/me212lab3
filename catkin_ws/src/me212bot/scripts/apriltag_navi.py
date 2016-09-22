#!/usr/bin/python

# 2.12 Lab 3 AprilTag Navigation: use AprilTag for current global robot (X,Y,Theta), and to navigate to target (X,Y,Theta)
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

from me212bot.msg import WheelVelCmd
from pr_apriltags.msg import AprilTagDetections
import helper

class ApriltagNavigator():
    def __init__(self, constant_vel = True):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        
        # self.velcmd_pub = ??
        
        if constant_vel:
            self.thread = threading.Thread(target = self.constant_vel_loop)
        else:
            self.thread = threading.Thread(target = self.navi_loop)
            
        self.thread.start()
        
        self.target_pose2d = [0.25, 0, np.pi]  # in front of the apriltag, facing apriltag
        
        rospy.sleep(1)
        
    def apriltag_callback(self, data):
        ## use apriltag pose detection to find where is the robot
        ##
        for detection in data.detections:
            if detection.id == 0: 
                pose_tag_base = helper.poseTransform(helper.pose2list(detection.pose),  homeFrame = '/camera', targetFrame = '/base_link', listener = self.listener)
                # pose_base_map = helper.poseTransform(??)
                # pubFrame(self.br, pose = pose_base_map, frame_id = '/base_link', parent_frame_id = '/map', npub = 1)


    def constant_vel_loop(self):
        while not rospy.is_shutdown() :
            ## Send a constant velocity command through /cmdvel topic
            rospy.sleep(0.01) 
        
        
    def navi_loop(self):
        # Create the velocity msg
        wv = WheelVelCmd()
        
        while not rospy.is_shutdown() :
            # 1. get robot pose
            robot_pose3d = helper.lookupTransformList('/map', '/base_link', self.listener)
            
            if robot_pose3d is None:
                print '1. Tag not in view, Stop'
                wv.desiredWV_R = 0  # right, left
                wv.desiredWV_L = 0
                self.velcmd_pub.publish(wv)  
                continue
            
            robot_position2d = robot_pose3d[0:2]
            target_position2d = target_pose2d[0:2]
            
            robot_yaw = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
            robot_pose2d = robot_position2d + [robot_yaw]
            
            ## 2. navigation policy
            # 2.1 if       in target pose region, stop
            # 2.2 else if  close to target position, turn to the target orientation
            # 2.3 else if  in the correct heading, go straight to the target position,
            # 2.4 else     turn in the direction of the target position
            
            rospy.sleep(0.01)
    
def main():
    rospy.init_node('me212_robot', anonymous=True)
    april_navi = ApriltagNavigator()
    rospy.spin()

if __name__=='__main__':
    main()
    
