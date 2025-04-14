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
        n_rotate = 20
        speed_rotate = 0.6
        forward_speed = 0.6
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
                speed = forward_speed
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

    # Parameters (TODO: put them in the control_tp2 function)
    K_goal = 2
    K_obs = -10000
    K_omega = 0.1
    K_V = 0.5
    phi_max = np.pi / 6
    d_seuil = 5
    d_quadratic = 100  # under this distance, we use a quadratic field to slow the robot
    d_safe = 100

    d_q_qgoal = np.sqrt(
        (current_pose[0] - goal_pose[0]) ** 2 + (current_pose[1] - goal_pose[1]) ** 2
    )

    if d_q_qgoal < d_seuil:
        # We stop moving, we're close enough from the goal
        return {"forward": 0, "rotation": 0}

    # Gradient in the odom frame
    if d_q_qgoal > d_quadratic:
        # Far from the goal, we use a conic field (linear gradient)
        attraction_gradient = (K_goal / d_q_qgoal) * (goal_pose[:2] - current_pose[:2])
    else:
        # Close to the goal, we use a quadratic field (slower gradient)
        # The coefficient is chosen to have a continuous behaviour between the 2 types of potential
        attraction_gradient = (K_goal / d_quadratic) * (
            goal_pose[:2] - current_pose[:2]
        )

    # Now we focus on the repulsion gradient
    # We only take the closest obstacle
    laser_dist = lidar.get_sensor_values()
    obs_index, d_q_qobs = np.argmin(laser_dist), np.amin(laser_dist)
    if d_q_qobs > d_safe:
        repulsion_gradient = np.zeros(2)
    else:
        obs_angle = (obs_index - 180) * np.pi / 180
        qobs = current_pose[:2] + d_q_qobs * np.array(
            [np.cos(obs_angle + current_pose[2]), np.sin(obs_angle + current_pose[2])]
        )
        repulsion_gradient = (
            (K_obs / (d_q_qobs**3))
            * ((1 / d_q_qobs) - (1 / d_safe))
            * (qobs - current_pose[:2])
        )

    # print(f"attraction:{attraction_gradient}, repulsion:{repulsion_gradient}")

    final_gradient = attraction_gradient + repulsion_gradient
    # Now we create the command

    # Angle between the gradient and the current direction:
    gradient_angle = np.arctan2(final_gradient[1], final_gradient[0])
    # print(f"final gradient: {final_gradient}")
    # print(f"gradient angle: {gradient_angle}")
    phi_r = gradient_angle - current_pose[2]
    # Ensure the angle is in [-pi, pi]
    phi_r = (phi_r + np.pi) % (2 * np.pi) - np.pi
    # print("current_robot_angle:", current_pose[2])
    # print("phi_r:", phi_r)
    command_rotation = K_omega * phi_r

    # Forward command:
    if np.abs(phi_r) < phi_max:
        command_forward = K_V * np.linalg.norm(final_gradient)
    else:
        command_forward = K_V * np.linalg.norm(final_gradient) * phi_max / np.abs(phi_r)
    command = {
        "forward": np.clip(command_forward, -1.0, 1.0),
        "rotation": np.clip(command_rotation, -1.0, 1.0),
    }

    return command
