"""A simple robotics navigation code including SLAM, exploration, planning"""

import cv2
import numpy as np
from occupancy_grid import OccupancyGrid


class TinySlam:
    """Simple occupancy grid SLAM"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

        # Origin of the odom frame in the map frame
        # At the beginning, the odom frame is the map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def _score(self, lidar, pose):
        """
        Computes the sum of log probabilities of laser end points in the map
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, position of the robot to evaluate, in world coordinates
        """
        # TODO for TP4

        score = 0

        return score

    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Compute corrected pose in map frame from raw odom pose + odom frame pose,
        either given as second param or using the ref from the object
        odom : raw odometry position
        odom_pose_ref : optional, origin of the odom frame if given,
                        use self.odom_pose_ref if not given
        """
        # TODO for TP4
        corrected_pose = odom_pose

        return corrected_pose

    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        odom : [x, y, theta] nparray, raw odometry position
        """
        # TODO for TP4

        best_score = 0

        return best_score

    def update_map(self, lidar, pose):
        """
        Bayesian map update with new observation
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, corrected pose in world coordinates
        """
        # TODO for TP3

        # First step
        # Get the lidar data and convert them to cartesian coordinates in the world frame
        lidarDist = lidar.get_sensor_values()
        lidarAngles = lidar.get_ray_angles()
        x0, y0, theta0 = pose

        # Slow version!
        # lidarCartesian = np.zeros((len(lidarDist), 2))
        # for i in range(len(lidarDist)):
        #     di = lidarDist[i]
        #     thetai = lidarAngles[i]
        #     x = x0 + di * np.cos(thetai + theta0)
        #     y = y0 + di * np.sin(theta0 + thetai)

        x = x0 + lidarDist * np.cos(lidarAngles + theta0)
        y = y0 + lidarDist * np.sin(lidarAngles + theta0)

        # Second step
        # Update the map for each point detected by the lidar
        #   First part: entire line, low probability

        for i in range(len(x)):
            self.grid.add_value_along_line(x0, y0, x[i], y[i], -0.1)
        #   Second part: detected cell, high probability
        self.grid.add_map_points(x, y, 1)

        # Third step
        # Threshold probabilities to avoid divergence
        self.grid.occupancy_map = self.grid.occupancy_map.clip(-40, 40)

    def compute(self):
        """Useless function, just for the exercise on using the profiler"""
        # Remove after TP1

        ranges = np.random.rand(3600)
        ray_angles = np.arange(-np.pi, np.pi, np.pi / 1800)

        # Poor implementation of polar to cartesian conversion
        points = []
        for i in range(3600):
            pt_x = ranges[i] * np.cos(ray_angles[i])
            pt_y = ranges[i] * np.sin(ray_angles[i])
            points.append([pt_x, pt_y])
