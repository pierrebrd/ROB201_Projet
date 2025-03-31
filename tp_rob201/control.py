"""A set of robotics control functions"""

import random
import numpy as np


def reactive_obst_avoid(lidar, counter):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    """
    # TODO for TP1

    laser_dist = lidar.get_sensor_values()  # get the lidar data
    new_counter = counter  # initialize new_counter with the current counter value

    version = 2

    if version == 1:
        # Problem : the robot follows the wall
        if laser_dist[180] < 50:
            speed = 0
            rotation_speed = 1
        else:
            speed = 1  # forward speed when no obstacles
            rotation_speed = 0  # rotation speed when no obstacles

    elif version == 2:
        # We want the robot to complete its rotation before going forward again, so that
        # it doesn't follow the walls
        n_rotate = 100
        speed_rotate = 1
        if new_counter > 0 and new_counter < n_rotate:  # We are rotating
            speed = 0
            rotation_speed = speed_rotate
            new_counter += 1
            if (
                new_counter == n_rotate
            ):  # We have finished rotating, we reset the new_counter
                new_counter = 0
        else:
            if laser_dist[180] < 50:
                new_counter = 1
                speed = 0
                rotation_speed = speed_rotate
            else:
                speed = 1
                rotation_speed = 0

    else:
        speed = 0
        rotation_speed = 0

    command = {"forward": speed, "rotation": rotation_speed}

    return command, new_counter


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Control using potential field for goal reaching and obstacle avoidance
    lidar : placebot object with lidar data
    current_pose : [x, y, theta] nparray, current pose in odom or world frame
    goal_pose : [x, y, theta] nparray, target pose in odom or world frame
    Notes: As lidar and odom are local only data, goal and gradient will be defined either in
    robot (x,y) frame (centered on robot, x forward, y on left) or in odom (centered / aligned
    on initial pose, x forward, y on left)
    """
    # TODO for TP2

    command = {"forward": 0, "rotation": 0}

    return command
