#!/usr/bin/env python3
import rospy
import math
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped

from scipy.spatial.transform import Rotation as R

from cares_msgs.msg import PlatformGoalGoal

from cares_lib_ros.utils import World
import cares_lib_ros.utils as utils

def scan_calibration(planning_link):
    target_pose = np.array([0.2, 0.6, -1.0])

    start_x = -0.2
    step_x  = 0.2
    end_x   = 0.2
    x_range = np.arange(start_x, end_x+step_x/2, step_x)

    start_y = 0.4
    step_y  = 0.10
    end_y   = 0.6
    y_range = np.arange(start_y, end_y+step_y, step_y)

    start_z =  -0.25
    step_z  =  0.05
    end_z   =  -0.10
    z_range = np.arange(start_z, end_z+step_z, step_z)

    print(f"Z: {len(z_range)} Y: {len(y_range)} X: {len(x_range)}")
    print(f"{x_range}")
    print(f"{y_range}")
    print(f"{z_range}")

    path = {}
    path['pathway'] = []
    path['target']  = PoseStamped(pose=utils.to_pose(target_pose, rpy=[0,0,0]))

    path['scanning'] = []
    roll = 10#degrees
    for z in z_range:
        for y in y_range:
            for x in x_range:
                pose = utils.look_at_pose(np.array([x, y, z]), target_pose, up=World.right)
    
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = planning_link
                pose_stamped.pose = pose
                path['pathway'].append(pose_stamped)
                path['scanning'].append(pose_stamped)
    return path

def tomato_path(planning_link):
    path = []
    start_x = -0.2
    step_x  = 0.1
    end_x   = 0.2

    y = 1.0
    # start_y = 1.1
    # step_y  = 0.1
    # end_y   = 1.1

    start_z = 0.0
    step_z  = 0.10
    end_z   = 0.6

    for z in np.arange(start_z, end_z+step_z, step_z):
        # for y in np.arange(start_y, end_y+step_y, step_y):
        for x in np.arange(start_x, end_x+step_x, step_x):
            # if z < 0:
            #     target_pose = np.array([x, 1.7, z+0.1])
            # else:
            #     target_pose = np.array([x, 1.7, z-0.1])
            target_pose = np.array([0, 2.0, z])
            pose = utils.look_at_pose(np.array([x, y, z]), target_pose, up=World.up)
            pose = utils.rotate_pose(pose, roll=-15)
        
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path.append(pose_stamped)

    return path

def arc_path(planning_link, target_z=0.1):
    path = {}

    radius = 0.41 # distance from the target we are scanning at

    # Target location relative to the planning_link
    target_y = 0.75
    target_z = target_z
    
    # marker pose to look from a given pose
    pose = utils.look_at_pose(np.array([-0.0, 0.29, -0.1]), np.array([0, 0.7, -1.0]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global'] = {}

    pose = utils.look_at_pose(np.array([0.0, 0.29, -0.15]), np.array([0.0, 0.85, -0.6]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global']['charuco'] = pose_stamped

    # Width we scan with steps between
    start_x = -0.4#-0.50
    step_x  = 0.1
    end_x   = 0.4# 0.5

    path['scanning'] = []

    # Arc we take to scan the target location - note 0 is striaght above, and -180 is straight below
    #         0
    #--------------------
    #       -180
    theta_range = np.linspace(-120, -50, 5)
    for x in np.arange(start_x, end_x+step_x, step_x):        
        for theta in theta_range:
            theta = utils.deg_rad(theta)
            y = radius * math.sin(theta) + target_y
            z = radius * math.cos(theta) + target_z

            pose = utils.look_at_pose(np.array([x, y, z]), np.array([x, target_y, target_z]), up=World.up)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path['scanning'].append(pose_stamped)
        theta_range = theta_range[::-1] 

    path['scanning'].reverse()
    return path

def arc_path_bottom(planning_link):
    path = {}

    radius = 0.4# distance from the target we are scanning at

    # Target location relative to the planning_link
    target_y = 0.7
    target_z = -0.25
    
    target_y_high = 0.735
    target_y_low = 0.8

    # marker pose to look from a given pose
    pose = utils.look_at_pose(np.array([-0.0, 0.29, -0.1]), np.array([0, 0.7, -1.0]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global'] = {}

    pose = utils.look_at_pose(np.array([0.0, 0.29 -0.15]), np.array([0.0, 0.85, -0.6]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global']['charuco'] = pose_stamped

    # Width we scan with steps between
    start_x = -0.4#-0.50
    step_x  = 0.1
    end_x   = 0.4# 0.5

    path['scanning'] = []

    # Arc we take to scan the target location - note 0 is striaght above, and -180 is straight below
    #         0
    #--------------------
    #       -180
    theta_range = np.linspace(-120, -50, 5)
    for x in np.arange(start_x, end_x+step_x, step_x):        
        for theta in theta_range:
            target_y = target_y_low if theta < -90 else target_y_high
            theta = utils.deg_rad(theta)
            y = radius * math.sin(theta) + target_y
            z = radius * math.cos(theta) + target_z

            look_at_z = z if theta < math.radians(-80) else target_z
            pose = utils.look_at_pose(np.array([x, y, z]), np.array([x, target_y, look_at_z]), up=World.up)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path['scanning'].append(pose_stamped)
        theta_range = theta_range[::-1] 

    path['scanning'].reverse()
    return path

def plane_path(planning_link):
    
    path = {}
    
    target_y = 0.6
    target_z = 0.3

    start_x = -0.5
    step_x  =  0.1
    end_x   =  0.5

    y = 0.4

    start_z = target_z - 0.2
    step_z  = 0.1
    end_z   = target_z + 0.3

    path['global'] = {}

    pose = utils.look_at_pose(np.array([-0.3, 0.4, 0.0]), np.array([-0.3, 0.8, -0.6]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global']['aruco'] = pose_stamped
    
    pose = utils.look_at_pose(np.array([0.3, 0.4, 0.0]), np.array([0.3, 0.8, -0.6]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global']['charuco'] = pose_stamped

    path['scanning'] = []
    z_range = np.arange(start_z, end_z+0.1, step_z)
    for i, z in enumerate(z_range):
        # for y in np.arange(start_y, end_y+step_y, step_y):
        x_range = np.arange(start_x, end_x+step_x, step_x) if not i%2 else np.arange(end_x, start_x-step_x, -step_x)

        for x in x_range:    
            # Flat
            pose = utils.look_at_pose(np.array([x, y, z]), np.array([x, y+0.2, z]), up=World.up)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path['scanning'].append(pose_stamped)
        z_range = z_range[::-1]
    return path

def nerf_path(planning_link):
    
    path = {}
    
    target_y = 0.6
    target_z = 0.25

    start_x = -0.3
    step_x  =  0.1
    end_x   =  0.3

    y = 0.4

    start_z = target_z - 0.2
    step_z  = 0.1
    end_z   = target_z + 0.2

    path['global'] = {}

    pose = utils.look_at_pose(np.array([-0.3, 0.4, 0.0]), np.array([-0.3, 0.8, -0.6]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global']['aruco'] = pose_stamped
    
    pose = utils.look_at_pose(np.array([0.3, 0.4, 0.0]), np.array([0.3, 0.8, -0.6]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global']['charuco'] = pose_stamped

    path['scanning'] = []
    z_range = np.arange(start_z, end_z+0.1, step_z)
    for i, z in enumerate(z_range):
        # for y in np.arange(start_y, end_y+step_y, step_y):
        x_range = np.arange(start_x, end_x+step_x, step_x) if not i%2 else np.arange(end_x, start_x-step_x, -step_x)

        for x in x_range:    
            # Flat
            pose = utils.look_at_pose(np.array([x, y, z]), np.array([0.0, 0.8, target_z]), up=World.up)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path['scanning'].append(pose_stamped)
        z_range = z_range[::-1]
    return path

def plane_path_test(planning_link):
    
    path = {}
    
    target_y = 0.6
    target_z = 0.1

    start_x = -0.0
    step_x  =  0.1
    end_x   =  0.0

    y = 0.4

    start_z = target_z
    step_z  = 0.1
    end_z   = target_z
    pose = utils.look_at_pose(np.array([0.0, 0.4, -0.15]), np.array([0.0, 0.85, -0.6]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global'] = {}
    path['global']['charuco'] = pose_stamped

    path['scanning'] = []
    z_range = np.arange(start_z, end_z+0.1, step_z)
    for i, z in enumerate(z_range):
        # for y in np.arange(start_y, end_y+step_y, step_y):
        x_range = np.arange(start_x, end_x+step_x, step_x) if not i%2 else np.arange(end_x, start_x-step_x, -step_x)

        for x in x_range:    
            # Flat
            pose = utils.look_at_pose(np.array([x, y, z]), np.array([x, y+0.2, z]), up=World.up)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path['scanning'].append(pose_stamped)
        z_range = z_range[::-1]
    return path

def plane_path_bot(planning_link):
    
    path = {}
    
    target_y = 0.
    target_z = -0.25

    start_x = -0.4
    step_x  =  0.1
    end_x   =  0.5
    
    y = 0.4

    start_z = target_z 
    step_z  = 0.1
    end_z   = target_z

    pose = utils.look_at_pose(np.array([0.0, 0.4, 0.0]), np.array([0, 0.7, -1.0]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global'] = pose_stamped

    path['scanning'] = []
    z_range = np.arange(start_z, end_z+step_z, step_z)
    z_range = [target_z]
    for x in np.arange(start_x, end_x+step_x, step_x):
        # for y in np.arange(start_y, end_y+step_y, step_y):
        for z in z_range:

            # Flat
            pose = utils.look_at_pose(np.array([x, y, z]), np.array([x, target_y, target_z]), up=World.up)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path['scanning'].append(pose_stamped)
        z_range = z_range[::-1]
    return path

def test_path(planning_link):
    
    path = {}
    
    target_y =  0.7
    target_z = -0.5

    start_x =  0.0
    step_x  =  0.1
    end_x   =  0.1

    y = 0.4

    start_z = 0.2
    step_z  = 0.1
    end_z   = 0.4

    pose = utils.look_at_pose(np.array([0.0, 0.4, 0.3]), np.array([0, 0.7, -1.0]), up=World.up)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_link
    pose_stamped.pose = pose
    path['global'] = pose_stamped

    path['scanning'] = []
    z_range = np.arange(start_z, end_z+step_z, step_z)
    for x in np.arange(start_x, end_x+step_x, step_x):
        # for y in np.arange(start_y, end_y+step_y, step_y):
        for z in z_range:    
            # Flat
            pose = utils.look_at_pose(np.array([x, y, z]), np.array([x, target_y, target_z]), up=World.up)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = planning_link
            pose_stamped.pose = pose
            path['scanning'].append(pose_stamped)
        z_range = z_range[::-1]
    return path

class PathFactory(object):
    def create_path(self, path_id, planning_link):
        if path_id == 0:
            return plane_path(planning_link)
        elif path_id == 1:
            return scan_point(planning_link)
        elif path_id == 2:
            return scan_calibration(planning_link)
        elif path_id == 3:
            return tomato_path(planning_link)
        elif path_id == 4:
            return arc_path(planning_link)
        elif path_id == 5:
            return test_path(planning_link)
        elif path_id == 6:
            return plane_path_bot(planning_link)
        elif path_id == 7:
            return nerf_path(planning_link)
        elif path_id == 8:
            return plane_path_test(planning_link)
        elif path_id == 9:
            return arc_tower(planning_link, False)
        elif path_id == 10:
            return arc_tower(planning_link, True)
        elif path_id == 11:
            return arc_path_bottom(planning_link)
        elif path_id == 12:
            return arc_path(planning_link, target_z=0.2)
        
        return []

def main():
    import open3d as o3d
    pose        = np.array([0,0,0])
    target_pose = np.array([0.0,1.0,0])

    print("Pose")
    print(pose)
    print("Target Pose")
    print(target_pose)
    pose = utils.look_at_pose(pose, target_pose, up=World.up)
    print("Pose - go")
    print(pose)

    a_q = utils.quaternion_to_array(pose.orientation)
    a_q = np.asarray([a_q[3],a_q[0],a_q[1], a_q[2]])
    rotmat = o3d.geometry.get_rotation_matrix_from_quaternion(a_q)
    print("Rotation Matrix")
    print(rotmat)

if __name__ == '__main__':
    main()
