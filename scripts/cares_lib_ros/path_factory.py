#! /usr/bin/env python
import rospy
import math
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs

r_2_d = 180 / math.pi

def normalise(p1):
    length = math.sqrt(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2])
    p1[0] = p1[0] / length
    p1[1] = p1[1] / length
    p1[2] = p1[2] / length

def crossproduct(p1, p2, cp):
    cp[0] = p1[1] * p2[2] - p1[2] * p2[1]
    cp[1] = p1[2] * p2[0] - p1[0] * p2[2]
    cp[2] = p1[0] * p2[1] - p1[1] * p2[0]


def quaternion2matrix(q, m):
    xx = q.orientation.x * q.orientation.x
    xy = q.orientation.x * q.orientation.y
    xz = q.orientation.x * q.orientation.z
    xw = q.orientation.x * q.orientation.w

    yy = q.orientation.y * q.orientation.y
    yz = q.orientation.y * q.orientation.z
    yw = q.orientation.y * q.orientation.w

    zz = q.orientation.z * q.orientation.z
    zw = q.orientation.z * q.orientation.w

    m[0][0] = 1 - 2 * ( yy + zz )
    m[0][1] = 2 * ( xy - zw )
    m[0][2] = 2 * ( xz + yw )

    m[1][0] = 2 * ( xy + zw )
    m[1][1] = 1 - 2 * ( xx + zz )
    m[1][2] = 2 * ( yz - xw )

    m[2][0] = 2 * ( xz - yw )
    m[2][1] = 2 * ( yz + xw )
    m[2][2] = 1 - 2 * ( xx + yy )

    m[0][3] = m[1][3] = m[2][3] = m[3][0] = m[3][1] = m[3][2] = 0
    m[3][3] = 1


def matrix2quaternion(a, q):
    #matrix to quaternion
    trace = a[0][0] + a[1][1] + a[2][2]
    if( trace > 0 ):
        s = 0.5 / math.sqrt(trace+ 1.0)
        q.orientation.w = 0.25 / s
        q.orientation.x = ( a[2][1] - a[1][2] ) * s
        q.orientation.y = ( a[0][2] - a[2][0] ) * s
        q.orientation.z = ( a[1][0] - a[0][1] ) * s
    else:
        if ( a[0][0] > a[1][1] and a[0][0] > a[2][2] ):
        
            s = 2.0 * math.sqrt( 1.0 + a[0][0] - a[1][1] - a[2][2])
            q.orientation.w = (a[2][1] - a[1][2] ) / s
            q.orientation.x = 0.25 * s
            q.orientation.y = (a[0][1] + a[1][0] ) / s
            q.orientation.z = (a[0][2] + a[2][0] ) / s
        elif (a[1][1] > a[2][2]):
            s = 2.0 * math.sqrt( 1.0 + a[1][1] - a[0][0] - a[2][2])
            q.orientation.w = (a[0][2] - a[2][0] ) / s
            q.orientation.x = (a[0][1] + a[1][0] ) / s
            q.orientation.y = 0.25 * s
            q.orientation.z = (a[1][2] + a[2][1] ) / s
        else:
            s = 2.0 * math.sqrt( 1.0 + a[2][2] - a[0][0] - a[1][1] )
            q.orientation.w = (a[1][0] - a[0][1] ) / s
            q.orientation.x = (a[0][2] + a[2][0] ) / s
            q.orientation.y = (a[1][2] + a[2][1] ) / s
            q.orientation.z = 0.25 * s


# NOTE ISSUES IF X[1] i.e. pose y == target y
def look_at_point(pose_goal, marker_pose):
    #Z is optical axis of camera, i.e. vector from camera pointing at maker
    X = [marker_pose.position.x - pose_goal.position.x, marker_pose.position.y - pose_goal.position.y, marker_pose.position.z - pose_goal.position.z]         
    Y = [0.0, 0.0, 0.0]

    #vectors are orthogonal, therefore dot product is zero
    Y[0] = pose_goal.position.x
    Y[2] = pose_goal.position.z + 0.1
    Y[1] = (-Y[2]*X[2] - Y[0]*X[0])/X[1]

    #cross product of Y and Z gives X
    Z = [0.0, 0.0, 0.0]
    crossproduct(X,Y,Z)

    normalise(X)
    normalise(Y)
    normalise(Z)
    rot_matrix = np.zeros((4,4))
    #Note that this is now confusing because I have switch
    rot_matrix[0][0] = X[0]
    rot_matrix[1][0] = X[1]
    rot_matrix[2][0] = X[2]
    rot_matrix[0][1] = Y[0]
    rot_matrix[1][1] = Y[1]
    rot_matrix[2][1] = Y[2]
    rot_matrix[0][2] = Z[0]
    rot_matrix[1][2] = Z[1]
    rot_matrix[2][2] = Z[2]
    rot_matrix[3][0] = 0
    rot_matrix[3][1] = 0
    rot_matrix[3][2] = 0
    rot_matrix[3][3] = 1
    # print(rot_matrix)
    matrix2quaternion(rot_matrix, pose_goal)

def scan_point():
    path = []
    
    target_pose = Pose()
    target_pose.position.x = 0.0
    target_pose.position.y = 1.7
    target_pose.position.z = 0.7
    print("Target Pose")
    print(target_pose)

    start_x = -0.3
    step_x  = 0.1
    end_x   = 0.3

    start_y = 0.7
    step_y  = 0.1
    end_y   = 0.7

    start_z = 0.7
    step_z  = 0.1
    end_z   = 1.2

    for z in np.arange(start_z, end_z+step_z, step_z):
        for y in np.arange(start_y, end_y+step_y, step_y):
            for x in np.arange(start_x, end_x+step_x, step_x):
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z

                look_at_point(pose, target_pose)
                
                path.append(pose)
    return path

def plane_path():
    path = []
    start_x = -0.3
    step_x  = 0.1
    end_x   = 0.3

    start_y = 0.7
    step_y  = 0.1
    end_y   = 0.7

    start_z = 0.7
    step_z  = 0.1
    end_z   = 1.2

    q = quaternion_from_euler(0, 0, 0)
    # q = quaternion_from_euler(-60.0/180.0*math.pi, 0, 0)
    
    for z in np.arange(start_z, end_z+step_z, step_z):
        for y in np.arange(start_y, end_y+step_y, step_y):
            for x in np.arange(start_x, end_x+step_x, step_x):
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z

                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                
                path.append(pose)
    return path

class PathFactory(object):
    def create_path(self, path_id):
        if path_id == 0:
            return plane_path()
        elif path_id == 1:
            return scan_point()
        return plane_path()

scan_point()
