#! /bin/python3.10

"""! @brief A Star supporting material for the 2023/24 4B12 individual project."""
##
# @file AStar.py
#
# @brief Impliments a simply AStar algorithm.
#
# @section author_astar Author(s)
# - Created by Patrick Lynch.

import csv
import math
import numpy as np


def compute_bezier_points(control_points):
    """
    Compute points on a Bézier curve given control points, with the number of points
    dynamically adjusted based on the total path length.
    
    :param control_points: List of tuples (x, y) representing control points.
    :return: List of tuples (x, y) representing points on the Bézier curve.
    """
    def distance(p1, p2):
        """Calculate Euclidean distance between two points."""
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    # Calculate the approximate path length
    path_length = sum(distance(control_points[i], control_points[i + 1]) for i in range(len(control_points) - 1))
    
    if path_length < 10:
        num_points = max(int(path_length / 0.1), 5) 
    else:
        num_points = max(int(path_length / 0.1), 50)  # Ensure at least 50 points for smoothness

    n = len(control_points) - 1
    bezier_points = []

    def bezier_interp(t):
        """Calculate the point on the Bézier curve for a given t"""


        x = sum(
            binomial_coeff(n, i) * ((1 - t) ** (n - i)) * (t ** i) * p[0]
            for i, p in enumerate(control_points)
        )
        y = sum(
            binomial_coeff(n, i) * ((1 - t) ** (n - i)) * (t ** i) * p[1]
            for i, p in enumerate(control_points)
        )
        return (x, y)
    
    def binomial_coeff(n, k):
        """Calculate the binomial coefficient 'n choose k'."""
        from math import factorial
        return factorial(n) / (factorial(k) * factorial(n - k))
    
    for i in range(num_points):
        t = i / float(num_points - 1)

        bezier_points.append(bezier_interp(t))
    
    return bezier_points


def distBetweenPoints(x1, y1, x2, y2):
    """! Takes the x and y coridanates of two points and returns the distance between them
    @param x1 **float**: The x coordinate of point 1
    @param y1 **float**: The y coordinate of point 1 
    @param x2 **float**: The x coordinate of point 2 
    @param y2 **float**: The y coordinate of point 2 
    @return **float**: The distance between the two points indicated by the input parameters
    """
    return(math.sqrt(pow(x2-x1,2)+pow(y2-y1,2)))

def createNodesList(occupancy_grid):
    """! Function used by the aStar function. 4B12 students will not have to interact with this function directly"""
    nodes = []
    row_counter = 0
    for row in occupancy_grid:
        nodes.append([])
        for element in row:
            occupied = int(element)
            g_cost = ""
            h_cost = ""
            f_cost = ""
            parent = ""
            nodes[row_counter].append({"o": occupied, "g":g_cost, "h":h_cost, "f":f_cost, "p":parent})
        row_counter += 1
    return nodes

def findLowestFCost(open_nodes_list, nodes):
    """! Function used by the aStar function. 4B12 students will not have to interact with this function directly"""
    success = False
    lowest_f_cost = 1000
    node_with_lfc = []
    for open_node in open_nodes_list:
        if nodes[open_node["row"]][open_node["column"]]["f"] < lowest_f_cost:
            lowest_f_cost = nodes[open_node["row"]][open_node["column"]]["f"]
            node_with_lfc = open_node
    if node_with_lfc == []:
        print("Error, no node with lfc")
    else:
        success = True
    return node_with_lfc, success

def aStar(robot_x, robot_y, goal_x, goal_y, occupancy_grid):

    """! Takes a start position, end position and an occupancy grid and returns a list of waypoints leading from the start position to the end position
    @param robot_x **float**: The x coordinate of the start position relative to the global cartesian coordinate frame
    @param robot_y **float**: The y coordinate of the start position relative to the global cartesian coordinate frame
    @param goal_x **float**: The x coordinate of the end position relative to the global cartesian coordinate frame
    @param goal_y **float**: The y coordinate of the end position relative to the global cartesian coordinate frame
    @param occupancy_grid **list**: A 2D occupancy grid in the form of a 2D python list, i.e. [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]]
    @return **list**: A list of waypoints which leads from the start position to the end position i.e. [[1, 2], [2, 2], [3, 3], [4, 3]]
    @return **boolean**: Indicator of the status of the search for a solution, i.e. did it successfully find a path or not
    """

    nodes = createNodesList(occupancy_grid)
    success = False

    goal_pos = {"x": goal_x, "y": goal_y}
    goal_node = {"row": int(goal_pos["y"]/0.1), "column": int(goal_pos["x"]/0.1)}
    start_pos = {"x": robot_x, "y": robot_y}
    start_node = {"row": int(start_pos["y"]/(0.1)), "column": int(start_pos["x"]/(0.1))}

    open_nodes_list = []
    closed_nodes_list = []
    open_nodes_list.append({"row": start_node["row"], "column":start_node["column"]})

    nodes[start_node["row"]][start_node["column"]]["g"] = 0
    dist_start_goal = distBetweenPoints(start_pos["x"], start_pos["y"], goal_pos["x"], goal_pos["y"])
    nodes[start_node["row"]][start_node["column"]]["h"] = dist_start_goal
    nodes[start_node["row"]][start_node["column"]]["f"] = dist_start_goal

    goal_reached = False

    while not goal_reached:
        success = True
        current_node, f_cost_success = findLowestFCost(open_nodes_list, nodes)
        if not f_cost_success:
            print("Failed to find Solution, this could be related to your occupancy grid and/or the 'buffer'")
            success = False
            break
        open_nodes_list.remove({"row": current_node["row"], "column":current_node["column"]})
        closed_nodes_list.append({"row": current_node["row"], "column": current_node["column"]})

        if current_node["row"] == goal_node["row"] and current_node["column"] == goal_node["column"]:
            goal_reached = True
            break

        neighbour_nodes_list = [{"row": current_node["row"]+1, "column": current_node["column"]}, 
                                {"row": current_node["row"], "column": current_node["column"]+1}, 
                                {"row": current_node["row"]-1, "column": current_node["column"]}, 
                                {"row": current_node["row"], "column": current_node["column"]-1}]

        for neig_node in neighbour_nodes_list:
            if neig_node not in closed_nodes_list and nodes[neig_node["row"]][neig_node["column"]]["o"] == 0:
                path_to_neighbour = nodes[current_node["row"]][current_node["column"]]["g"]+0.1
                neig_worth_exploring = False
                if nodes[neig_node["row"]][neig_node["column"]]["g"] != "":
                    if path_to_neighbour < nodes[neig_node["row"]][neig_node["column"]]["g"]:
                        neig_worth_exploring = True
                elif neig_node not in open_nodes_list:
                    neig_worth_exploring = True
                if neig_worth_exploring:
                    neig_pos = {"x": neig_node["column"]*0.1, "y": neig_node["row"]*0.1}
                    dist_neig_goal = distBetweenPoints(neig_pos["x"], neig_pos["y"], goal_pos["x"], goal_pos["y"])
                    nodes[neig_node["row"]][neig_node["column"]]["g"] = path_to_neighbour
                    nodes[neig_node["row"]][neig_node["column"]]["h"] = dist_neig_goal
                    nodes[neig_node["row"]][neig_node["column"]]["f"] = path_to_neighbour + dist_neig_goal
                    nodes[neig_node["row"]][neig_node["column"]]["p"] = current_node
                    if neig_node not in open_nodes_list:
                        open_nodes_list.append(neig_node)

    full_path = False
    path_grid = []
    path_coord = []
    path_grid.append(goal_node)
    current_node = goal_node

    if success:
        while not full_path:

            if current_node["row"] == start_node["row"] and current_node["column"] == start_node["column"]:
                full_path = True
                break

            path_grid.append(nodes[current_node["row"]][current_node["column"]]["p"])
            current_node = nodes[current_node["row"]][current_node["column"]]["p"]
        for element in path_grid:
            path_coord.append([element["column"]*0.1, element["row"]*0.1])

        path_coord = list(reversed(path_coord))
        success = True

    return path_coord, success
