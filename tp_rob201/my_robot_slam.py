"""
Robot controller definition
Complete controller including SLAM, planning, path following
"""

import numpy as np

from place_bot.entities.robot_abstract import RobotAbstract
from place_bot.entities.odometer import OdometerParams
from place_bot.entities.lidar import LidarParams

from tiny_slam import TinySlam

from control import (
    potential_field_control,
    reactive_obst_avoid,
)  # We import the control functions we need
from occupancy_grid import OccupancyGrid
from planner import Planner


# Definition of our robot controller
class MyRobotSlam(RobotAbstract):
    """A robot controller including SLAM, path planning and path following"""

    def __init__(
        self,
        lidar_params: LidarParams = LidarParams(),
        odometer_params: OdometerParams = OdometerParams(),
    ):
        # Passing parameter to parent class
        super().__init__(
            should_display_lidar=False,
            lidar_params=lidar_params,
            odometer_params=odometer_params,
        )

        # step counter to deal with init and display
        self.counter = 0

        # Init SLAM object
        # Here we cheat to get an occupancy grid size that's not too large, by using the
        # robot's starting position and the maximum map size that we shouldn't know.
        size_area = (1400, 1000)
        robot_position = (439.0, 195)
        self.occupancy_grid = OccupancyGrid(
            x_min=-(size_area[0] / 2 + robot_position[0]),
            x_max=size_area[0] / 2 - robot_position[0],
            y_min=-(size_area[1] / 2 + robot_position[1]),
            y_max=size_area[1] / 2 - robot_position[1],
            resolution=2,
        )

        self.tiny_slam = TinySlam(self.occupancy_grid)
        self.planner = Planner(self.occupancy_grid)

        # storage for pose after localization
        self.corrected_pose = np.array([0, 0, 0])

        # My personal parameters
        self.goal = [-400, -20, 0]
        self.trajectory = np.matrix(np.array([0, 0])).T
        # print(self.trajectory.shape)

    def control(self):
        """
        Main control function executed at each time step
        """
        if self.counter == 0:
            self.tiny_slam.update_map(self.lidar(), self.corrected_pose)

        # TP4 : Let's start by correcting the odometry pose
        best_score = self.tiny_slam.localise(self.lidar(), self.odometer_values())
        print("best score", best_score)
        self.corrected_pose = self.tiny_slam.get_corrected_pose(self.odometer_values())

        # Maybe we should not not update the map if the score is too low, but rather not update the posiiton of the robot/odom (by restoring the old one)
        if best_score > 500:  # TODO : improve the value
            # Update the lidar map
            self.tiny_slam.update_map(self.lidar(), self.corrected_pose)

        # Update trajectory # TODO : doesnt work
        self.trajectory = np.hstack(
            (self.trajectory, np.matrix(self.corrected_pose[:2]).T)
        )
        # print(self.trajectory)

        command, qobs = self.control_tp2()

        if self.counter % 10 == 0:
            self.tiny_slam.grid.display_cv(self.corrected_pose, qobs, self.trajectory)
        self.counter += 1

        return command  # We choose wich control function we want to use

    def control_tp1(self):
        """
        Control function for TP1
        Control funtion with minimal random motion
        """
        # self.tiny_slam.compute()  # TODO : enlever ?

        # Compute new command speed to perform obstacle avoidance
        command, new_counter = reactive_obst_avoid(
            self.lidar(), self.counter
        )  # We want to avoid obstacles (here defined by the lidar)
        self.counter = new_counter
        return command

    def control_tp2(self):
        """
        Control function for TP2
        Main control function with full SLAM, random exploration and path planning
        """
        # Compute new command speed to perform obstacle avoidance
        command, qobs = potential_field_control(
            self.lidar(), self.corrected_pose, self.goal
        )

        return command, qobs
