#! /usr/bin/env python
import numpy as np

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import numpy as np

from platform_msgs.msg import PlatformGoalGoal

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




class World:
    forward = np.array([0.0, 1.0, 0.0])
    right = np.array([1.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])


def look_dir_matrix(forward, up=World.up, frame="body"):
    forward = normalize(forward)
    left = normalize(np.cross(up, forward))
    up = np.cross(forward, left)

    if frame == "body":
        return np.stack([forward, -left, -up], axis=1)  
    elif frame == "optical":
        return np.stack([left, up, forward], axis=1) 
    else:
        return f"look_dir_matrix: unknown frame {frame}"
    


def grid(size=(8, 8), scale=(0.5, 0.5)):
    sx, sy = scale
    h, w = size
    x, y = np.meshgrid(np.linspace(-sx, sx, w), np.linspace(-sy, sy, h))
    return np.stack([x, y], axis=-1)



def scan_plane(points_2d, up=World.up, forward=World.forward):
    right = -normalize(np.cross(up, forward))
    points_3d = points_2d[...,0:1] * right + points_2d[...,1:2] * up

    m = look_dir_matrix(forward, up)
    return [to_pose(p, matrix=m) for p in points_3d.view(-1, 3)]




class PathFactory(object):

    @staticmethod
    def plane_forward(size=(8, 8), scale=(0.5, 0.5)):
        return scan_plane(grid(size, scale))
