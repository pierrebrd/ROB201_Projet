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
        lidarDist = lidar.get_sensor_values()
        lidarAngles = lidar.get_ray_angles()
        x, y, theta = pose

        # Remove the points that are at the max range
        validIndices = np.where(lidarDist < lidar.max_range)
        lidarDist = lidarDist[validIndices]
        lidarAngles = lidarAngles[validIndices]

        # Convert the lidar data to world coordinates
        x_lidar = x + lidarDist * np.cos(lidarAngles + theta)
        y_lidar = y + lidarDist * np.sin(lidarAngles + theta)

        # Convert to map coordinates in the occupancy grid
        x_map, y_map = self.grid.conv_world_to_map(x_lidar, y_lidar)

        # Remove the points that are outside the map
        select = np.logical_and(
            np.logical_and(x_map >= 0, x_map < self.grid.x_max_map),
            np.logical_and(y_map >= 0, y_map < self.grid.y_max_map),
        )
        x_map = x_map[select]
        y_map = y_map[select]

        score = np.sum(self.grid.occupancy_map[x_map, y_map])

        # print("score", score)

        return score

    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Compute corrected pose in map frame from raw odom pose + odom frame pose,
        either given as second param or using the ref from the object
        odom_pose : raw odometry position
        odom_pose_ref : optional, origin of the odom frame if given,
                        use self.odom_pose_ref if not given
        """
        if odom_pose_ref is None:
            odom_pose_ref = self.odom_pose_ref

        x_oref, y_oref, theta_oref = odom_pose_ref
        x_o, y_o, theta_o = odom_pose
        d_o = np.sqrt(x_o**2 + y_o**2)
        alpha_o = np.arctan2(y_o, x_o)

        x = x_oref + d_o * np.cos(theta_oref + alpha_o)
        y = y_oref + d_o * np.sin(theta_oref + alpha_o)
        theta = theta_oref + theta_o

        corrected_pose = np.array([x, y, theta])

        return corrected_pose

    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        raw_odom_pose : [x, y, theta] nparray, raw odometry position
        """

        # Compute the score for the current self.odom_pose_ref
        best_score = self._score(lidar, self.get_corrected_pose(raw_odom_pose))

        counter_no_improvement = 0
        while counter_no_improvement < 100:
            sigma_xy = 0.2
            sigma_theta = 0.02
            # Generate random offset from a Gaussian distribution
            offset = np.random.normal(0, [sigma_xy, sigma_xy, sigma_theta])
            candidate_odom_pose_ref = self.odom_pose_ref + offset
            # print("offset", offset)
            # print("candidate_odom_pose_ref", candidate_odom_pose_ref)
            candidate_corrected_pose = self.get_corrected_pose(
                raw_odom_pose, candidate_odom_pose_ref
            )
            candidate_score = self._score(lidar, candidate_corrected_pose)

            # Compare with the best score
            if candidate_score > best_score:
                # We update the best score and the odom reference
                best_score = candidate_score
                self.odom_pose_ref = candidate_odom_pose_ref
                # Reset counter
                counter_no_improvement = 0
            else:
                counter_no_improvement += 1

        return best_score

    def update_map(self, lidar, pose):
        """
        Bayesian map update with new observation
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, corrected pose in world coordinates
        """

        # First step
        # Get the lidar data and convert them to cartesian coordinates in the world frame
        lidarDist = lidar.get_sensor_values()
        lidarAngles = lidar.get_ray_angles()
        x0, y0, theta0 = pose

        x = x0 + lidarDist * np.cos(lidarAngles + theta0)
        y = y0 + lidarDist * np.sin(lidarAngles + theta0)
        # x is a list, not the x from the world frame coordinates of the robot

        # Second step
        # Update the map for each point detected by the lidar
        #   First part: entire line, low probability

        for i in range(len(x)):
            self.grid.add_value_along_line(x0, y0, x[i], y[i], -1.255)
        #   Second part: detected cell, high probability
        self.grid.add_map_points(x, y, 10)

        # Third step
        # Threshold probabilities to avoid divergence
        self.grid.occupancy_map = self.grid.occupancy_map.clip(-40, 40)

        score = self._score(lidar, pose)
