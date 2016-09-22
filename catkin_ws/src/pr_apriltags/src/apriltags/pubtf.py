#!/usr/bin/env python

from pr_apriltags.msg import AprilTagDetections
from tf import *
import ik.roshelper
from ik.helper import *
import rospy
import os
import json

rospy.init_node("pubtf")

br = TransformBroadcaster()

with open(os.environ['APC_BASE'] + '/catkin_ws/src/apc_config/object_data/objectApriltagTransform.json') as data_file:    
    Poses = json.load(data_file)

Dict = {}
for key, value in Poses.iteritems():
    Dict[value["id"]] = key
    print value["id"], key
    
def callback(data):
    
    for detection in data.detections:
        if detection.id not in Dict:
            print '[Callback AprilTag] detection.id not known', detection.id
            continue
        else:
            print '[Callback AprilTag] found object', Dict[detection.id]
            ik.roshelper.pubFrame(br, pose = ik.roshelper.pose2list(detection.pose), frame_id = '/tag/'+Dict[detection.id], parent_frame_id = '/realsense_rgb_optical_frame', npub=1)
            pos = Poses[Dict[detection.id]]["pose"][0:3]
            quat = Poses[Dict[detection.id]]["pose"][3:7]
            ik.roshelper.pubFrame(br, pose = xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(pos,quat))), frame_id = '/obj/'+Dict[detection.id], parent_frame_id = '/tag/'+Dict[detection.id], npub=1)
        #if detection.id == 0: #What is this?
        #    ik.roshelper.pubFrame(br, pose = ik.roshelper.pose2list(detection.pose), frame_id = 'tag/command_hooks', parent_frame_id = '/tmp_realsense', npub=1)
            #rospy.sleep(1)
            
            

def callback2(data):
    pass
rospy.Subscriber("/pr_apriltags/detections", AprilTagDetections, callback)

rospy.spin()
