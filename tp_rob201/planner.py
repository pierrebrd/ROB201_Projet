"""
Planner class
Implementation of A*
"""

import numpy as np
import heapq

from occupancy_grid import OccupancyGrid


class Planner:
    """Simple occupancy grid Planner"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

        # Origin of the odom frame in the map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def get_neighbors(self, current_cell):
        """
        Returns the 8 neighbors of current_cell, as well as the move cost
        """
        x, y = current_cell
        neighbors = []

        dx_list = [0]
        dy_list = [0]
        if x > 0:
            dx_list.append(-1)
        if x < self.grid.x_max_map - 1:
            dx_list.append(1)
        if y > 0:
            dy_list.append(-1)
        if y < self.grid.y_max_map - 1:
            dy_list.append(1)

        for dx in dx_list:
            for dy in dy_list:
                if dx == 0 and dy == 0:
                    continue
                # We append the neighbor and the move cost
                neighbors.append(((x + dx, y + dy), 1 if dy * dx == 0 else 1.414))
        return neighbors

    def heuristic(self, cell_1, cell_2):
        """
        Returns the heuristic distance between two cells. The heuristic is the Euclidean distance
        """
        return np.sqrt((cell_1[0] - cell_2[0]) ** 2 + (cell_1[1] - cell_2[1]) ** 2)

    def plan(self, start, goal):
        """
        Compute a path using A*, recompute plan if start or goal change
        start : [x, y, theta] nparray, start pose in world coordinates (theta unused)
        goal : [x, y, theta] nparray, goal pose in world coordinates (theta unused)
        The path is returned in world coordinates
        """

        # We start by creating a copy of the map with bigger obstacles
        obstacles_increment = 10
        padded_map = np.pad(
            self.grid.occupancy_map, pad_width=obstacles_increment, constant_values=0
        )
        big_obstacles_map = np.zeros_like(self.grid.occupancy_map)
        print("max x: ", self.grid.x_max_map)
        for x in range(self.grid.x_max_map):
            print("x", x)
            for y in range(self.grid.y_max_map):
                window = padded_map[
                    x : x + 2 * obstacles_increment + 1,
                    y : y + 2 * obstacles_increment + 1,
                ]
                big_obstacles_map[x, y] = np.max(window)

        # First, we convert the start and goal to map coordinates
        start_cell = self.grid.conv_world_to_map(start[0], start[1])
        goal_cell = self.grid.conv_world_to_map(goal[0], goal[1])

        # In our heap, to have a priority queue, we will store the
        # estimated total cost, the cost so far, the current cell and the path to that cell
        queue = [
            (self.heuristic(start_cell, goal_cell), 0, start_cell, [start_cell])
        ]  # list of cells to explore
        heapq.heapify(queue)  # Don't know if this is needed
        visited = set()

        while queue != []:
            # Get the cell with lower estimated total cost
            estimated_total_cost, cost_so_far, current_cell, path = heapq.heappop(queue)

            if current_cell == goal_cell:
                # We found the goal, we return the path
                return [
                    self.grid.conv_map_to_world(cell[0], cell[1]) for cell in path
                ]  # In map coordinates

            if current_cell in visited:
                continue
            visited.add(current_cell)

            for neighbor, move_cost in self.get_neighbors(current_cell):
                if (
                    neighbor in visited
                    or big_obstacles_map[neighbor[0], neighbor[1]] > 0
                ):
                    continue
                new_cost = cost_so_far + move_cost
                estimated_total_cost = new_cost + self.heuristic(neighbor, goal_cell)
                heapq.heappush(
                    queue, (estimated_total_cost, new_cost, neighbor, path + [neighbor])
                )
        return None  # No path found if we reach here

    def explore_frontiers(self):
        """Frontier based exploration"""
        goal = np.array([0, 0, 0])  # frontier to reach for exploration
        return goal
