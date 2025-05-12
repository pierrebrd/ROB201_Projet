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
        self.trajectory = np.array([0, 0])
        # print(self.trajectory.shape)

        # TP5 : new parameters
        # 0 : explore to the goal using potential field
        # 1 : go back to initial position using A*
        self.navigation_mode = 0
        self.planned_path = None
        self.planned_path_map = None
        self.path_increment = 20
        self.current_path_objective_index = self.path_increment

    def control(self):
        """
        Main control function executed at each time step
        """

        # TP4 : Let's start by correcting the odometry pose
        best_score = self.tiny_slam.localise(self.lidar(), self.odometer_values())
        # print("counter :", self.counter)
        print("best score", best_score)
        self.corrected_pose = self.tiny_slam.get_corrected_pose(self.odometer_values())
        # print("corrected pose", self.corrected_pose)

        # Maybe we should not not update the map if the score is too low, but rather not update the posiiton of the robot/odom (by restoring the old one)
        if best_score > 3000 or self.counter < 40:  # TODO : improve the value
            # Update the lidar map
            self.tiny_slam.update_map(self.lidar(), self.corrected_pose)

        # Update trajectory # TODO : doesnt work
        self.trajectory = np.vstack((self.trajectory, self.corrected_pose[:2]))
        # print(self.trajectory)

        if self.navigation_mode == 0:
            # Potential field control until we reach the goal
            command, qobs = self.control_tp2()
            if command == None:
                # We have arrived to the goal
                self.navigation_mode = 1  # Switch the mode
                self.planned_path = np.array(
                    self.planner.plan(self.corrected_pose, np.array([0, 0, 0]))
                )
                command = {"forward": 0, "rotation": 0}
            if self.counter % 10 == 0:
                self.tiny_slam.grid.display_cv(
                    self.corrected_pose, self.goal, self.trajectory
                )

        # TP5 : we use the A* algorithm
        elif self.navigation_mode == 1:
            # We go back to the initial point
            command, qobs = potential_field_control(
                self.lidar(),
                self.corrected_pose,
                self.planned_path[self.current_path_objective_index],
                self.navigation_mode,
            )
            while (
                command == None
                and self.current_path_objective_index < len(self.planned_path) - 1
            ):
                # We reached the intermediate objective
                self.current_path_objective_index += self.path_increment
                command, qobs = potential_field_control(
                    self.lidar(),
                    self.corrected_pose,
                    self.planned_path[self.current_path_objective_index],
                    self.navigation_mode,
                )
            if self.counter % 10 == 0:
                self.tiny_slam.grid.display_cv(
                    self.corrected_pose,
                    self.planned_path[self.current_path_objective_index],
                    self.planned_path,
                )

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
        result = potential_field_control(
            self.lidar(), self.corrected_pose, self.goal, self.navigation_mode
        )

        return result
