import rospy
import tf
import numpy as np
import tf.transformations as tfm

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped
import helper

nTfRetry = 1
retryTime = 0.05

def poselist2pose(poselist):
    pose = Pose()
    pose.position.x = poselist[0]
    pose.position.y = poselist[1]
    pose.position.z = poselist[2]
    pose.orientation.x = poselist[3]
    pose.orientation.y = poselist[4]
    pose.orientation.z = poselist[5]
    pose.orientation.w = poselist[6]
    return pose

def pose2poselist(pose):
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def invPoselist(poselist):
    return xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(poselist)))

def pubFrame(br, pose=[0,0,0,0,0,0,1], frame_id='obj', parent_frame_id='map', npub=1):
    if len(pose) == 7:
        ori = tuple(pose[3:7])
    elif len(pose) == 6:
        ori = tfm.quaternion_from_euler(*pose[3:6])
    else:
        print 'Bad length of pose'
        return None
    
    pos = tuple(pose[0:3])
    
    for j in range(npub):
        br.sendTransform(pos, ori, rospy.Time.now(), frame_id, parent_frame_id)
        rospy.sleep(0.01)

def lookupTransform(lr, sourceFrame, targetFrame):
    for i in range(nTfRetry):
        try:
            t = rospy.Time(0)
            (trans,rot) = lr.lookupTransform(sourceFrame, targetFrame, t)
            if lr.getLatestCommonTime(sourceFrame, targetFrame) < (rospy.Time.now() - rospy.Duration(1)):
                return None
            return list(trans) + list(rot)
        except:
            print '[lookupTransform] failed to transform targetFrame %s sourceFrame %s, retry %d' % (targetFrame, sourceFrame, i)
            rospy.sleep(retryTime)
    return None

def transformPose(lr, pose, sourceFrame, targetFrame):
    _pose = PoseStamped()
    _pose.header.frame_id = sourceFrame
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = tfm.quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()
    
    _pose.pose.position.x = pose[0]
    _pose.pose.position.y = pose[1]
    _pose.pose.position.z = pose[2]
    _pose.pose.orientation.x = pose[3]
    _pose.pose.orientation.y = pose[4]
    _pose.pose.orientation.z = pose[5]
    _pose.pose.orientation.w = pose[6]
    
    for i in range(nTfRetry):
        try:
            t = rospy.Time(0)
            _pose.header.stamp = t
            _pose_target = lr.transformPose(targetFrame, _pose)
            p = _pose_target.pose.position
            o = _pose_target.pose.orientation
            return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        except: 
            print '[transformPose] failed to transform targetFrame %s sourceFrame %s, retry %d' % (targetFrame, sourceFrame, i)
            rospy.sleep(retryTime)
            
    return None

def xyzquat_from_matrix(matrix):
    return tfm.translation_from_matrix(matrix).tolist() + tfm.quaternion_from_matrix(matrix).tolist()

def matrix_from_xyzquat(arg1, arg2=None):
    return matrix_from_xyzquat_np_array(arg1, arg2).tolist()

def matrix_from_xyzquat_np_array(arg1, arg2=None):
    if arg2 is not None:
        translate = arg1
        quaternion = arg2
    else:
        translate = arg1[0:3]
        quaternion = arg1[3:7]

    return np.dot(tfm.compose_matrix(translate=translate) ,
                   tfm.quaternion_matrix(quaternion))

def cross2d(a, b):
    return a[0]*b[1] - a[1]*b[0]

def diffrad(a,b):
    diff = (a-b)
    while diff < -np.pi:
        diff += 2*np.pi
    while diff > np.pi:
        diff -= 2*np.pi
    return diff

