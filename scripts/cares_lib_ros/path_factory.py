#!/usr/bin/env python3
import rospy
import math
import numpy as np

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import String

from typing import Tuple
from scipy.spatial.transform import Rotation as R

from platform_msgs.msg import PlatformGoalGoal

import cares_lib_ros.utils as utils

def to_pose(position, matrix=None, quaternion=None, rpy=None):
    assert (matrix is not None or quaternion is not None or rpy is not None)

    rotation = R.from_matrix(matrix) if matrix is not None\
        else R.from_quat(quaternion) if quaternion is not None\
        else R.from_euler('xyz', rpy)

    qx, qy, qz, qw = rotation.as_quat()
    x, y, z = position

    return Pose(
        position=Point(x, y, z), 
        orientation=Quaternion(qx, qy, qz, qw)
    )

def normalize(v):
    norm = np.linalg.norm(v)
    return v if norm == 0 else v / norm  

def direction(origin, target):
  return normalize(target - origin)

class World:
    forward = np.array([0.0, 1.0, 0.0])
    right = np.array([1.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])


class Body:
    forward = np.array([1.0, 0.0, 0.0])
    right = np.array([0.0, -1.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])


class Optical:
    forward = np.array([0.0, 0.0, 1.0])
    right = np.array([1.0, 0.0, 0.0])
    up = np.array([0.0, -1.0, 0.0])


def look_basis(forward, up=World.up, frame="body"):
    """
        Used to build 'look at' type rotation matrices from forwards and up vector.
        Up is not the exact 'up' direction, but recalculated after computing right. 
        Coordinate frame convention is by Ros naming conventions.
        :up np.array        up vector(3d) in world coordinates
        :forward np.array   forwards vector(3d) in world coordinates to look along
        :frame str          coordinate frame convention, either 'body' | 'optical'
    """
    forward = normalize(forward)
    left = normalize(np.cross(up, forward))
    up = np.cross(forward, left)

    if frame == "body":
        return np.stack([forward, -left, -up], axis=1)  
    elif frame == "optical":
        return np.stack([left, up, forward], axis=1) 
    else:
        return f"look_dir_matrix: unknown frame {frame}"
    

def look_around(position, target, orbit_pos, frame="body"):
    """
        Used to orbit around a port where the camera 'up' vector is pointing away from 'orbit_pos'. 
        The target should not point at the orbit position.
        :position np.array   camera position vector(3d)
        :target np.array     forwards vector(3d) in world coordinates to look along
        :orbit_pos np.array  position where the camera will point 'down' towards 
        :frame str           coordinate frame convention, either 'body' | 'optical'
    """

    forwards = normalize(target - position)
    orbit_dir = normalize(orbit_pos - position)

    assert np.abs(1 - np.dot(orbit_dir, forwards)) > 1e-6, "camera should not point at the orbit position" 
    return look_basis(forwards, up=-orbit_dir, frame=frame)

def look_at(position, target, up=World.up, frame="body"):
    """
        Canonical look at, formed by a camera position and target along with an up vector. 
        Giving a camera with a consistant perceived up direction (i.e. no roll). The target
        should not be in the direction of the up vector.
        :position np.array   camera position vector(3d)
        :target np.array     forwards vector(3d) in world coordinates to look along
        :up np.array         the world space 'up' vector 
        :frame str           coordinate frame convention, either 'body' | 'optical'
    """
    return look_basis(target - position, up=up, frame=frame)


def look_at_pose(position, target, up=World.up, frame="body"):
    return to_pose(position, look_at(position, target, up, frame))

def scan_calibration(planning_link):
    
    target_pose = np.array([0.0, 1.1, -1.0])

    #6
    start_x = -0.25
    step_x  = 0.10
    end_x   = 0.25

    #3
    start_y = 0.95
    step_y  = 0.10
    end_y   = 1.15

    #3
    start_z = -0.3
    step_z  = 0.1
    end_z   = -0.1

    path = {}
    path['pathway'] = []
    path['target']  = PoseStamped(pose = to_pose(target_pose, rpy=[0,0,0]))

    for z in np.arange(start_z, end_z+step_z, step_z):
        for y in np.arange(start_y, end_y+step_y, step_y):
            for x in np.arange(start_x, end_x+step_x, step_x):
                    
                    # target_pose = np.array([0, 1.45, -0.3])
                    # target_pose = np.array([0, 1.45, z])
                    # target_pose = np.array([x, 1.45, z])
                    pose = look_at_pose(np.array([x, y, z]), target_pose, up=World.right)
                    
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = planning_link
                    pose_stamped.pose = pose
                    path['pathway'].append(pose_stamped)
    return path

def plane_path(planning_link):
    path = []
    start_x = -0.2
    step_x  = 0.1
    end_x   = 0.2

    y = 1.1
    # start_y = 1.1
    # step_y  = 0.1
    # end_y   = 1.1

    start_z = -0.15
    step_z  = 0.05
    end_z   = 0.55

    for z in np.arange(start_z, end_z+step_z, step_z):
        # for y in np.arange(start_y, end_y+step_y, step_y):
        for x in np.arange(start_x, end_x+step_x, step_x):

            target_pose = np.array([0, 1.70, z])
            pose = look_at_pose(np.array([x, y, z]), target_pose, up=World.up)
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path.append(pose_stamped)

    return path

class PathFactory(object):
    def create_path(self, path_id, planning_link):
        if path_id == 0:
            return plane_path(planning_link)
        elif path_id == 1:
            return scan_point(planning_link)
        elif path_id == 2:
            return scan_calibration(planning_link)
        return []

def main():
    import open3d as o3d
    pose        = np.array([0,0,0])
    target_pose = np.array([0.0,1.0,0])

    print("Pose")
    print(pose)
    print("Target Pose")
    print(target_pose)
    pose = look_at_pose(pose, target_pose, up=World.up)
    print("Pose - go")
    print(pose)

    a_q = utils.quaternion_to_array(pose.orientation)
    a_q = np.asarray([a_q[3],a_q[0],a_q[1], a_q[2]])
    rotmat = o3d.geometry.get_rotation_matrix_from_quaternion(a_q)
    print("Rotation Matrix")
    print(rotmat)

if __name__ == '__main__':
    main()