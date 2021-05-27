#!/usr/bin/env python3
import rospy
import roslib
import tf
import math 
import time
import yaml
from glob import glob
import open3d as o3d

from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros
import tf2_geometry_msgs
from platform_msgs.msg import PlatformGoalGoal
import ros_numpy

from math import pi
import numpy as np
import cv2

from datetime import datetime

from os.path import expanduser
home = expanduser("~")

def create_pose_msg(x, y, z, rpy=None, quaternion=None):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    if rpy is not None:
        quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    elif quaternion is None:
        quaternion = [0.0,0.0,0.0,1.0]
        
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

def create_goal_msg(pose, action, link_id, move_mode=0):
    # Creates a goal to send to the action server.
    pose_goal = PlatformGoalGoal()
    pose_goal.command = action
    pose_goal.target_pose = pose
    pose_goal.link_id.data = link_id
    pose_goal.move_mode = move_mode 
    return pose_goal

def save_transform(filename, transform):
    translation_dic = {}
    translation_dic["x"] = transform.transform.translation.x
    translation_dic["y"] = transform.transform.translation.y
    translation_dic["z"] = transform.transform.translation.z
    
    rotation_dic = {}
    rotation_dic["x"] = transform.transform.rotation.x
    rotation_dic["y"] = transform.transform.rotation.y
    rotation_dic["z"] = transform.transform.rotation.z
    rotation_dic["w"] = transform.transform.rotation.w
    
    transform_dic = {}
    transform_dic["parent_frame_id"] = transform.header.frame_id
    transform_dic["child_frame_id"]  = transform.child_frame_id
    transform_dic["translation"] = translation_dic
    transform_dic["rotation"] = rotation_dic
    
    with open(filename, "w") as file:
        documents = yaml.dump(transform_dic, file)

def read_transform(filepath):
    with open(filepath) as file:
        t_map = yaml.load(file, Loader=yaml.Loader)
        t = TransformStamped()
        t.header.frame_id = t_map["parent_frame_id"]
        t.child_frame_id  = t_map["child_frame_id"]
        t.transform.translation.x = t_map["translation"]["x"]
        t.transform.translation.y = t_map["translation"]["y"]
        t.transform.translation.z = t_map["translation"]["z"]
        t.transform.rotation.x = t_map["rotation"]["x"]
        t.transform.rotation.y = t_map["rotation"]["y"]
        t.transform.rotation.z = t_map["rotation"]["z"]
        t.transform.rotation.w = t_map["rotation"]["w"]
        return t

def load_transforms(path):
    files = glob(path)
    files.sort()
    print("Found {} transforms at {}".format(len(tuple(files)), path))
    
    transforms = []
    for i, file in enumerate(files):
        try:
            transform = read_transform(file)
            transforms.append(transform)
        except KeyError as e:
            print("error at ", i)
            print(e)
            raise

    return transforms, files

def loadImages(path):
    images = []
    files = glob(path)
    files.sort()
    print("Found {} images at {}".format(len(tuple(files)), path))

    for i, file in enumerate(files):
        image = cv2.imread(file)
        images.append(image)
    return images, files

def rectify_image(image_left, image_right, stereo_info):
    return rectify_images([images_right], [image_right], stereo_info)

def rectify_images(images_left, images_right, stereo_info):
    w = stereo_info.left_info.width
    h = stereo_info.left_info.height

    K1 = np.reshape(stereo_info.left_info.K, (3,3))
    R1 = np.reshape(stereo_info.left_info.R, (3,3))
    P1 = np.reshape(stereo_info.left_info.P, (3,4))
    D1 = np.array(stereo_info.left_info.D)
    
    K2 = np.reshape(stereo_info.right_info.K, (3,3))
    R2 = np.reshape(stereo_info.right_info.R, (3,3))
    P2 = np.reshape(stereo_info.right_info.P, (3,4))
    D2 = np.array(stereo_info.right_info.D)
    
    # Q = np.reshape(stereo_info.Q, (4,4))
    # R = np.reshape(stereo_info.R_left_right,(3,3))
    # T = np.array(stereo_info.T_left_right)
    
    left_map_x, left_map_y   = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w,h), cv2.CV_32FC1)
    right_map_x, right_map_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w,h), cv2.CV_32FC1)
    images_left_rectified = []
    images_right_rectified = []
    for i in range(len(images_left)):
        left_rectified  = cv2.remap(images_left[i], left_map_x, left_map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)    
        right_rectified = cv2.remap(images_right[i], right_map_x, right_map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        images_left_rectified.append(left_rectified)
        images_right_rectified.append(right_rectified) 
    return images_left_rectified, images_right_rectified


def pointCloud2Open3D(point_cloud):
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(point_cloud)
    pc = pc.flatten()
    pc = ros_numpy.point_cloud2.split_rgb_field(pc)
    
    xyzrgb = np.zeros((pc.shape[0],6))
    xyzrgb[:,0]=pc['x']
    xyzrgb[:,1]=pc['y']
    xyzrgb[:,2]=pc['z']
    xyzrgb[:,3]=pc['r']
    xyzrgb[:,4]=pc['g']
    xyzrgb[:,5]=pc['b']
    xyzrgb = xyzrgb[~np.isnan(xyzrgb).any(axis=1)]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyzrgb[:,:3])
    pcd.colors = o3d.utility.Vector3dVector(np.divide(xyzrgb[:,3:],255.0))
    return pcd

def transform_open3d(xyzrgb, transform):
    translation = transform.transform.translation
    x = translation.x
    y = translation.y
    z = translation.z
    rotation = transform.transform.rotation
    qx = rotation.x
    qy = rotation.y
    qz = rotation.z
    qw = rotation.w

    rotmat_np = np.asarray([qw,qx,qy,qz])
    rotmat = o3d.geometry.get_rotation_matrix_from_quaternion(rotmat_np)
    xyzrgb.rotate(rotmat,center=(0, 0, 0))

    translation = np.asarray([x,y,z])
    xyzrgb.translate(translation)
    return xyzrgb