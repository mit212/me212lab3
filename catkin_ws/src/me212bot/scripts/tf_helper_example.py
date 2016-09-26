#!/usr/bin/python

# 2.12 Lab 3 tf examples
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, poselist2pose, invPoselist

def main():
    rospy.init_node('apriltag_navi', anonymous=True)
    lr = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    poselist_tag_map = [0,0,0.44,0.5,0.5,0.5,0.5]
    print 'poselist_tag_map', poselist_tag_map, '\n'
    
    pose_tag_map = poselist2pose(poselist_tag_map)
    print 'poselist2pose(poselist_tag_map):\n', pose_tag_map, '\n'
    
    poselist_map_tag = invPoselist(poselist_tag_map)
    print 'invPoselist(poselist_tag_map):', poselist_map_tag, '\n'
    
    poselist_tag_map_bylookup = lookupTransform(lr, sourceFrame = '/apriltag', targetFrame = '/map')
    print "lookupTransform(poselist_tag_map, sourceFrame = '/tag', targetFrame = '/map'):", poselist_tag_map_bylookup, '\n'
    
    poselist_map_tag_bylookup = lookupTransform(lr, sourceFrame = '/map', targetFrame = '/apriltag')
    print "lookupTransform(poselist_tag_map, sourceFrame = '/map', targetFrame = '/apriltag'):", poselist_map_tag_bylookup, '\n'
    
    poselist_base_tag = [0,0,1,0,0,0,1]
    poselist_base_map = transformPose(lr, poselist_base_tag, sourceFrame = '/apriltag', targetFrame = '/map')
    print "transformPose(poselist_tag_map, sourceFrame = '/apriltag', targetFrame = '/map'):", poselist_base_map, '\n'
    
    for i in xrange(100):
        pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')
        rospy.sleep(0.1)
    

if __name__=='__main__':
    main()
    
