#! /usr/bin/env python
import numpy as np
from scipy.spatial.transform import Rotation as R

from . import utils

def plane_grid(width=0.5, w_divs=4, height=0.5, h_divs=4,
    right=[1, 0, 0], up=[0, 0, 1]):

    xs=np.linspace(-width/2, width/2, w_divs)
    ys=np.linspace(-height/2, height/2, h_divs)

    xv, yv = np.meshgrid(xs, ys)
    points = np.stack([xv * right, yv * up], axis=1)
    return points

class World:
    forward = np.array([0.0, 1.0, 0.0])
    backward = np.array([0.0, -1.0, 0.0])

    left = np.array([-1.0, 0.0, 0.0])
    right = np.array([1.0, 0.0, 0.0])

    up = np.array([0.0, 0.0, 1.0])
    down = np.array([0.0, 0.0, -1.0])


def look_dir_matrix(dir, up=World.up, roll=0, frame="body"):
    forward = utils.normalize(dir)
    left = utils.normalize(np.cross(up, forward))
    up = np.cross(forward, left)

    if frame == "body":
        return np.stack([forward, -left, -up], axis=1)  @ R.from_euler('x', roll).as_matrix()
    elif frame == "optical":
        return np.stack([left, up, forward], axis=1) @ R.from_euler('z', roll).as_matrix()
    else:
        return f"look_dir_matrix: unknown frame {frame}"
    

class PathFactory(object):
    def __init__(self, home_pos, target, frame="body", up=World.up):
        self.frame = frame

        self.target = np.array(target)
        self.home_pos = np.array(home_pos)

        self.up = np.array(up)
        self.forward = utils.normalize(target - np.array(home_pos))

        self.home = self.look_dir_pose(position=self.home_pos, dir=self.forward)  
        self.calibrate = self.randomized_orbit()

    @staticmethod
    def scan_forward(frame="body"):
        return PathFactory(frame=frame, up=World.up, home_pos=[0, 0.6, 0.5], target=[0, 1.2, 0.5])

    @staticmethod
    def scan_down(frame="body"):
        return PathFactory(frame=frame, up=World.forward, home_pos=[0, 0.6, 0.5], target=[0, 0.0, 0.5])


    def look_at_matrix(self, position, target,  roll=0):  
        return look_dir_matrix(np.array(target) - np.array(position), up=self.up, roll=roll, frame=self.frame)

    def look_at_pose(self, position, target,  roll=0):
        return utils.to_pose(position, matrix=self.look_at_matrix(position, target,  roll=roll))

    def look_dir_pose(self, position, dir, roll=0):
        return utils.to_pose(position, matrix=look_dir_matrix(dir, roll=roll))

    def randomized_orbit(self, angle=utils.deg_rad(45), roll_range=utils.deg_rad(30), distance_range=(0.3, 0.8), seed=0):
        
        rng = np.random.default_rng(seed=seed)    
        while True:
            v = utils.sample_cone(-np.array(self.forward), angle, rng)
            roll = rng.uniform(-roll_range, roll_range)

            sample_pos = self.target + v * np.random.uniform(*distance_range)
            yield self.look_at_pose(sample_pos, self.target, roll=roll)




