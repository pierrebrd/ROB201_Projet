<!-- LTeX: language=en-US -->

# ROB201 Project

This git repository *ensta-rob201* is the starting point for your work during the ROB201 project/course. The online, up-to-date version of this code is available at: [GitHub repository *ensta-rob201*](https://github.com/emmanuel-battesti/ensta-rob201)

To start working on your ROB201 project, you will have to install this repository (which will install required dependencies). The [INSTALL.md](INSTALL.md) file contains installation tips for Ubuntu and Windows. The code will work also on MacOS, with minimal adaptation of the Ubuntu installation instructions.

# Place-Bot

*ensta-rob201* use the **Place-Bot** simulator: [**Place-Bot** GitHub repository](https://github.com/emmanuel-battesti/place-bot). It will be installed automatically with the above procedure, but it is strongly recommended to read the [*Place-Bot* documentation](https://github.com/emmanuel-battesti/place-bot#readme).


# My project 

You can see the evolution of my project by looking at the different commits I made.

When running the file `main.py`, the robot will move to a predefined goal using a potential field control. The parameters can be adjusted in the `potential_field_control` function in `control.py`. The robot also dynamically creates a map of its environment using its Lidar sensor. It uses this map to correct the value of the odometry to know its position.

When the robot reaches the goal, it will go back to its initial position. For that, we use the A* algorithm to find the shortest path to the initial position, using the map that the robot created when exploring the environment. To follow the path, we use the potential field control with different parameters and a goal that is adjusted dynamically to be at a certain distance from the robot. 