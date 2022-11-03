#!/usr/bin/env python3
import rospy
import roslib
import tf
import math
import time
import yaml
from glob import glob
from natsort import natsorted, ns
import open3d as o3d

from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros
import tf2_geometry_msgs
from cares_msgs.msg import PlatformGoalGoal

from sensor_msgs.msg import Image, CameraInfo
from cares_msgs.msg import StereoCameraInfo

from math import pi
import numpy as np
import numpy.lib.recfunctions as rfn
import ros_numpy

import cv2

from datetime import datetime

from os.path import expanduser
home = expanduser("~")

def deg_rad(a):
    return np.pi/180.0 * a

def rad_deg(a):
    return 180.0/np.pi * a

def quaternion_to_array(q):
    return np.array([q.x, q.y, q.z, q.w])

def create_pose_msg(x, y, z, rpy=None, rpy_deg=None, quaternion=None):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    if rpy is not None:
        quaternion = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    elif rpy_deg is not None:
        quaternion = quaternion_from_euler(deg_rad(rpy_deg[0]), deg_rad(rpy_deg[1]), deg_rad(rpy_deg[2]))
    elif quaternion is None:
        quaternion = [0.0,0.0,0.0,1.0]

    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

def create_pose_stamped_msg(x, y, z, frame_id, time_stamp=None, rpy=None, rpy_deg=None, quaternion=None):
    pose = PoseStamped()
    pose.header.stamp = rospy.get_rostime() if time_stamp is None else time_stamp
    pose.header.frame_id = frame_id
    pose.pose = create_pose_msg(x, y, z, rpy, rpy_deg, quaternion)
    return pose

# related to PlatformGoal action - TODO move out of here to somewhere else or rename/refactor else where
def create_goal_msg(pose, action, link_id):
    # Creates a goal to send to the action server.
    pose_goal = PlatformGoalGoal()
    pose_goal.command = action
    pose_goal.target_pose = pose
    pose_goal.link_id.data = link_id
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
    files = natsorted(glob(path))
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
    files = natsorted(glob(path))
    print("Found {} images at {}".format(len(tuple(files)), path))

    for i, file in enumerate(files):
        image = cv2.imread(file, cv2.IMREAD_UNCHANGED)
        images.append(image)
    return images, files

def read_camerainfo_map(s_map):
    camera_info = CameraInfo()
    camera_info.header.frame_id = s_map["header"]["frame_id"]
    camera_info.height = s_map["height"]
    camera_info.width  = s_map["width"]
    camera_info.distortion_model = s_map["distortion_model"]
    camera_info.D = s_map["D"]
    camera_info.K = s_map["K"]
    camera_info.R = s_map["R"]
    camera_info.P = s_map["P"]
    camera_info.binning_x = s_map["binning_x"]
    camera_info.binning_y = s_map["binning_y"]

    camera_info.roi.x_offset   = s_map["roi"]["x_offset"]
    camera_info.roi.y_offset   = s_map["roi"]["y_offset"]
    camera_info.roi.height     = s_map["roi"]["height"]
    camera_info.roi.width      = s_map["roi"]["width"]
    camera_info.roi.do_rectify = s_map["roi"]["do_rectify"]
    return camera_info

def load_camerainfo(filepath):
    with open(filepath) as file:
        s_map = yaml.load(file, Loader=yaml.Loader)
        return read_camerainfo_map(s_map)

def load_stereoinfo(filepath):
    with open(filepath) as file:
        s_map = yaml.load(file, Loader=yaml.Loader)
        # print(s_map)
        stereo_info = StereoCameraInfo()
        stereo_info.header.frame_id = s_map["header"]["frame_id"]

        stereo_info.left_info  = read_camerainfo_map(s_map["left_info"])
        stereo_info.right_info = read_camerainfo_map(s_map["right_info"])

        stereo_info.Q = s_map["Q"]
        stereo_info.T_left_right = s_map["T_left_right"]
        stereo_info.R_left_right = s_map["R_left_right"]

        return stereo_info

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

    if 'rgba' in pc.dtype.names:
        pc = rfn.rename_fields(pc, {'rgba': 'rgb'})

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

def transform_pose_stamped(pose, transform):
    return tf2_geometry_msgs.do_transform_pose(pose, transform)

def transform_pose(pose, transform):
    pose = PoseStamped(pose=pose)
    return transform_pose_stamped(pose, transform).pose

def transform_point(point, transform):
    return tf2_geometry_msgs.do_transform_point(point, transform)

def transform_xyz(x, y, z, transform):
    point = PointStamped(point=Point(x=x,y=y,z=z))
    point = transform_point(point, transform)
    return point.point.x, point.point.y, point.point.z

def natsorted_list(files, remove_files=[]):
    return natsorted(list(set(glob(files)) - set(remove_files)))

def random_colours(num_colours=1):
    return [np.random.choice(range(256), size=3).tolist() for _ in range(num_colours)]

def create_pcd(rgb_image, depth_image, camera_info, depth_scale=1, depth_trunc=1000.0):
    # Creating RGBD Image
    o3d_depth_image = o3d.geometry.Image(depth_image)
    o3d_rgb_image   = o3d.geometry.Image(cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_rgb_image, o3d_depth_image, depth_trunc=depth_trunc, depth_scale=depth_scale, convert_rgb_to_intensity=False)

    # Extracting camera params
    height, width  = np.array(rgb_image).shape[:2]

    K = camera_info.K
    fx = K[0]
    fy = K[4]
    cx = K[2]
    cy = K[5]

    pinholeCamera = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    # Generating PCD
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinholeCamera)

    return pcd

def create_instance_pcds(image, depth, masks, camera_info, depth_scale=1, depth_trunc=3.0):
    # Generate instance masked depths
    masked_depths = []
    for mask in masks:
        bool_mask = ((mask[:, :, 0] != 0) & (mask[:, :, 1] != 0) & (mask[:, :, 2] != 0)).astype(bool)
        masked_depth = np.where(bool_mask, depth, 0)
        masked_depths.append(masked_depth)

    # Generate instance pcds
    instance_pcds = []
    for depth_mask in masked_depths:
        instance_pcd = create_pcd(image, depth_mask, camera_info) # create_pcd(image, depth_mask, camera_info.K)
        if instance_pcd.has_points():
            instance_pcds.append(instance_pcd)
    return instance_pcds

