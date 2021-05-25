#! /usr/bin/env python
import rospy
import math
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import Pose
import utils

import tf2_ros
import tf2_geometry_msgs


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


def plane_path_normal():
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

    # q = quaternion_from_euler(0, 0, 0)
    q = quaternion_from_euler(-60.0/180.0*math.pi, 0, 0)
    
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
        return plane_path()
