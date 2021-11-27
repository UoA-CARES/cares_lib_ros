#! /usr/bin/env python
import math
from typing import Tuple
import numpy as np

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import numpy as np

from platform_msgs.msg import PlatformGoalGoal

def normalize(v):
    norm = np.linalg.norm(v)
    return v if norm == 0 else v / norm  


def concat_lists(lists):
    r = []
    for list in lists:
        r.extend(list)
    return r


def direction(origin, target):
  return normalize(target - origin)


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


def transform_to_homog(transform):
  return rt_to_homog(*transform_to_rt(transform))

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

def offset_pose(pose:Pose, offset:np.array):
    pos = offset + point_to_array(pose.position)
    return Pose(
        position    = Point(*pos),
        orientation = pose.orientation
    )



def pose_to_homog(pose):
  return rt_to_homog(*pose_to_rt(pose))


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


def homog_to_pose(m):
  return to_pose(m[:3, 3], m[:3, :3])


def pose_to_rt(pose):
    q, t = pose_to_qt(pose)
    return R.from_quat(q).as_matrix(), t

def pose_to_qt(pose):
    return [
        quaternion_to_array(pose.orientation),
        point_to_array(pose.position)
    ]



def create_goal_msg(pose, action, link_id):
    # Creates a goal to send to the action server.
    return PlatformGoalGoal(
        command = action,
        target_poses = [pose],
        link_id = String(link_id))



def create_multi_goal_msg(poses, action, link_id, cartesian_path=False):
    # Creates a goal to send to the action server.
    return PlatformGoalGoal(
        command = action,
        target_poses = poses,
        cartesian_path=cartesian_path,
        link_id = String(link_id))

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


def mesh_grid_nd(size, scale):
    ranges = [np.linspace(-extent/2, extent/2, steps) for extent, steps in zip(scale, size)]
    axes = np.meshgrid(*ranges)
    return np.stack(axes, axis=-1)


def grid_2d(size=(8, 8), scale=(0.5, 0.5), snake=True):
    points = mesh_grid_nd(size, scale)
    if snake is True:  # reverse every second line to minimize movement
        points[1::2] = np.flip(points[1::2], axis=1)
    return points

def cube_points(size=(4, 4, 4), scale=(0.5, 0.5, 0.25), order=(0, 1, 2), snake=True):
    points = mesh_grid_nd(size, scale)
    points = points.transpose(*np.argsort(order), -1)
    if snake is True:  # reverse every second line and every second plane to minimize movement
        points[:, 1::2] = np.flip(points[:, 1::2], axis=(2))
        points[1::2] = np.flip(points[1::2], axis=(1, 2))

    return points



def on_plane(points_2d, origin, up=World.up, forward=World.forward):
    right = -normalize(np.cross(up, forward))
    return origin + points_2d[...,0:1] * right + points_2d[...,1:2] * up


def arc_points(origin=np.zeros(3), target=World.forward * 0.2, axis=World.right,
  arc_range=(-math.pi/4, math.pi/4), segments:int=8):


  forward = origin - target # pointing from target to origin
  up = np.cross(forward, normalize(axis)) # orthogonal to axis and forward

  angles = np.linspace(*arc_range, segments).reshape(-1, 1)
  return target + np.sin(angles) * up + np.cos(angles) * forward



def path_ortho(points_3d, up=World.up, forward=World.forward):
    """
        Scan  and computed from forward and up.
        :points_2d np.array   Nx2 2d points on a plane
        :up np.array          vector(3d) up vector
        :forward np.array     vector(3d) forward vector
    """
    m = look_basis(forward, up)
    return [to_pose(p, matrix=m) for p in points_3d.reshape(-1, 3)]


def path_look(points_3d, target, up=World.up):
    return [to_pose(p, matrix=look_at(p, target, up=up)) 
        for p in points_3d.reshape(-1, 3)]



def roll_variations(pose, angle_range=math.pi/2, stops=8, axis=Body.forward):
    q, t = pose_to_qt(pose)
    r = R.from_quat(q)

    angles = np.linspace(0, angle_range/2, stops//2).tolist()[1:]

    rotations = [[r * R.from_rotvec(axis * angle),  r * R.from_rotvec(-axis * angle)]
        for angle in angles]
        
    return [pose] + [to_pose(t, quaternion=R.as_quat(r)) for r in concat_lists(rotations)]


def snake_paths(pathways):
    return [ list(reversed(points)) if i % 2 > 0 else points 
        for i, points in enumerate(pathways)]


class PathFactory(object):

    @staticmethod
    def look_at(position, target, up=World.up):
        return to_pose(position, look_at(position, target, up))

    @staticmethod   
    def plane_forward(origin, size=(8, 8), scale=(0.5, 0.5)):
        points = on_plane(grid_2d(size, scale), origin)
        return path_ortho(points)


    @staticmethod 
    def single_arc(origin=np.zeros(3), segments=8, radius=0.2, arc_range=(-math.pi/4, math.pi/4), axes=World):
        print(origin)

        target = origin + axes.forward * radius
        points = arc_points(origin=origin, target=target, axis = axes.right, arc_range=arc_range, segments=segments)
        
        return path_look(points, target, up=axes.up)


    @staticmethod 
    def repeat_along(path, origin, axis_range, stops=4,  axis=World.right):
        poses = []

        for x in np.linspace(*axis_range, int(stops)):
            offset = origin + axis * x
            path_offset = [offset_pose(pose, offset) for pose in path]

            
            poses.append(path_offset)
        return poses


    @staticmethod 
    def arc_scan(origin, axis_range=(-0.1, 0.3), radius=0.2, segments=4, stops=4, arc_range=(-math.pi/4, math.pi/4), axes=World):
        arc = PathFactory.single_arc(segments=segments, radius=radius, arc_range=arc_range, axes=World)
        pose_arcs =  PathFactory.repeat_along(arc, origin, axis_range, stops, axes.right)

        return concat_lists(snake_paths(pose_arcs))


    @staticmethod   
    def calibration_scan(origin, target, size=(4, 4, 4), scale=(0.2, 0.1, 0.2), up=World.up):

        points = origin + cube_points(size, scale, order=(0, 2, 1))
        return path_look(points, target, up=up)