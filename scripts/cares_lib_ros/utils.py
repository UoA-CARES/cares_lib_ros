#!/usr/bin/env python3
from concurrent.futures.thread import ThreadPoolExecutor
from tqdm import tqdm
import yaml
import open3d as o3d

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import String
import numpy as np

from platform_msgs.msg import PlatformGoalGoal
import ros_numpy

from sensor_msgs.msg import CameraInfo
from cares_msgs.msg import StereoCameraInfo

import numpy as np
import cv2

from os.path import expanduser
home = expanduser("~")


def normalize(v):
    norm = np.linalg.norm(v)
    return v if norm == 0 else v / norm  


def point_to_array(point):
    return np.array([point.x, point.y, point.z])

def quaternion_to_array(q):
    return np.array([q.x, q.y, q.z, q.w])

def transform_to_qt(transform):
    return (
        quaternion_to_array(transform.rotation),
        point_to_array(transform.translation)
    )

def transform_to_rt(transform):
    return (
        R.from_quat(quaternion_to_array(transform.rotation)).as_matrix(),
        point_to_array(transform.translation)
    )

def transform_to_rt(transform):
    return (R.from_quat(quaternion_to_array(transform.rotation)).as_matrix(),
            point_to_array(transform.translation)
    )

def rt_to_homog(r, t):
  m = np.eye(4)
  m[:3, :3] = r
  m[:3, 3] = t
  return m

def deg_rad(a):
    return np.pi/180.0 * a 


def rad_deg(a):
    return 180.0/np.pi * a 


def sample_cone(dir, angle, rng=None):
    rng = rng or np.random.default_rng()
    z = rng.uniform(np.cos(angle), 1)
    phi = rng.uniform(0, 2.0 * np.pi)

    dir = normalize(dir)
    x = orthogonal(dir)
    y = np.cross(dir, x)

    sz = np.sqrt(1 - z*z)
    return (x * sz * np.cos(phi) + y * sz * np.sin(phi) + z * dir)



def orthogonal(v):
    """ Find an arbitrary vector orthogonal to v """
    not_v = np.array([1, 0, 0])
    if np.allclose(v, not_v):
        not_v = np.array([0, 1, 0])

    return np.cross(v, not_v)


def pose_to_qt(pose):
    return [
        quaternion_to_array(pose.orientation),
        point_to_array(pose.position)
    ]

def pose_to_rt(pose):
    return [
        R.from_quat(quaternion_to_array(pose.orientation)).as_matrix(),
        point_to_array(pose.position)
    ]


def to_pose(position, matrix=None, quaternion=None, rpy=None):
    assert (matrix is not None or quaternion is not None or rpy is not None)

    rotation = R.from_matrix(matrix) if matrix is not None\
        else R.from_quat(quaternion) if quaternion is not None\
        else R.from_euler(rpy)

    qx, qy, qz, qw = rotation.as_quat()
    x, y, z = position

    return Pose(
        position=Point(x, y, z), 
        orientation=Quaternion(qx, qy, qz, qw)
    )


def pose_to_rt(pose):
    q, t = pose_to_qt(pose)
    return R.from_quat(q).as_matrix(), t

def pose_to_qt(pose):
    return [
        quaternion_to_array(pose.orientation),
        point_to_array(pose.position)
    ]


def create_goal_msg(pose, action, link_id, move_mode=0):
    
    # Creates a goal to send to the action server.
    return PlatformGoalGoal(
        command = action,
        target_pose = pose,
        link_id = String(link_id),
        move_mode = move_mode)

def vector_dict(v):
    x, y, z = v
    return dict(x=x, y=y, z=z)

def save_vector(v):
    x, y, z = v.tolist()
    return dict(x=x, y=y, z=z)

def save_quaternion(q):
    x, y, z, w = q.tolist()
    return dict(x=x, y=y, z=z, w=w)

def load_vector(d):
    return np.array([d['x'], d['y'], d['z']])

def load_quaternion(d):
    return np.array([d['x'], d['y'], d['z'], d['w']])


def save_transform(filename, transform):
    q, t = transform_to_qt(transform.transform)

    transform_dict = dict(
        parent_frame_id = transform.header.frame_id,
        child_frame_id = transform.child_frame_id, 
        translation = save_vector(t),
        rotation = save_quaternion(q)
    )
   
    with open(filename, "w") as file:
        documents = yaml.dump(transform_dict, file)


def read_transform(filepath):
    with open(filepath) as file:
        t_map = yaml.load(file, Loader=yaml.Loader)
        t = TransformStamped()
        t.header.frame_id = t_map["parent_frame_id"]
        t.child_frame_id  = t_map["child_frame_id"]

        p = load_vector(t_map["translation"])
        q = load_quaternion(t_map["rotation"])

        t.transform.translation = Point(*p)
        t.transform.rotation = Quaternion(*q)
        return t

def load_transforms(filenames):    
    transforms = []
    for i, file in enumerate(filenames):
        try:
            transform = read_transform(file)
            transforms.append(transform)
        except KeyError as e:
            print(f"error reading {file}")
            print(e)
            raise

    return transforms


def load_images(filenames):
  with ThreadPoolExecutor() as executor:
    iter = executor.map(cv2.imread, filenames)
    return list(tqdm(iter, total=len(filenames)))

def load_stereoinfo(filepath):
    with open(filepath) as file:
        s_map = yaml.load(file, Loader=yaml.Loader)
        # print(s_map)
        stereo_info = StereoCameraInfo()
        stereo_info.header.frame_id = s_map["header"]["frame_id"]

        def load_camerainfo(s_map):
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

        stereo_info.left_info  = load_camerainfo(s_map["left_info"])
        stereo_info.right_info = load_camerainfo(s_map["right_info"])

        stereo_info.Q = s_map["Q"]
        stereo_info.T_left_right = s_map["T_left_right"]
        stereo_info.R_left_right = s_map["R_left_right"]

        return stereo_info

def rectify_image(image_left, image_right, stereo_info):
    return rectify_images([image_left], [image_right], stereo_info)

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