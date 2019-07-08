#!/usr/bin/env python
"""
ESE 680
Dijkstra's and A* toy example
Author: Hongrui Zheng
Everything planned in grid space
"""

import numpy as np
from numpy import linalg as LA
import math

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

class PathPlanner(object):
    def __init__(self):
        self.use_astar = False

        self.path = None

        # pub subs
        self.viz_pub = rospy.Publisher("planner_viz", Marker, queue_size=10)
        self.path_pub = rospy.Publisher("planner_path", Path, queue_size=10)
        self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_callback)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        map_metadata_msg = rospy.wait_for_message('/map_metadata', MapMetaData)
        self.map_resolution = map_metadata_msg.resolution
        self.map_width = map_metadata_msg.width
        self.map_height = map_metadata_msg.height
        self.origin_x = map_metadata_msg.origin.position.x
        self.origin_y = map_metadata_msg.origin.position.y
        map_msg = rospy.wait_for_message("/map", OccupancyGrid)
        # 1d_occgird = map_msg.data
        # 1D to 2D grid as ndarray
        self.occgrid = np.asarray(map_msg.data)
        self.occgrid = self.occgrid.reshape((self.map_height, self.map_width))
        self.uniform_cost = 1

        # TODO: start and goal should be in grid space
        self.start = None
        self.goal = None

    def coord_2_cell(self, x, y):
        col = (int)((x-self.origin_x)/self.map_resolution)
        row = (int)((y-self.origin_y)/self.map_resolution)
        return row*self.map_width+col

    def coord_2_rc(self, x, y):
        col = (int)((x-self.origin_x)/self.map_resolution)
        row = (int)((y-self.origin_y)/self.map_resolution)
        return (row, col)

    def rc_2_coord(self, r, c):
        coord = Point()
        coord.x = self.origin_x + c*self.map_resolution
        coord.y = self.origin_y + r*self.map_resolution
        return coord

    # def heuristic(self, x, y, goal_x, goal_y):
    #     # euclidean
    #     return math.sqrt((x-goal_x)**2+(y-goal_y)**2)

    def heuristic(self, neighbor, goal):
        # euclidean
        return math.sqrt((neighbor[0]-goal[0])**2 + (neighbor[1]-goal[1])**2)

    def astar_plan(self):
        # visited
        visited = []
        # frontier with f score
        frontier = {self.start: 0}
        # running cost for node
        g_score = {self.start: 0}
        # back track
        came_from = {}
        while len(frontier) > 0:
            print("current frontier length: ", len(frontier))
            current_node = min(frontier, key=frontier.get)
            if current_node == self.goal:
                print("path found")
                return self.reconstruct_path(came_from, current_node)

            current_node_value = frontier.pop(current_node)
            visited.append(current_node)

            neighbors = self.get_neighbors(current_node)
            for neighbor in neighbors:
                if neighbor in visited:
                    continue
                tentative_score = self.heuristic(neighbor, self.goal) + g_score[current_node] + self.uniform_cost
                if neighbor not in frontier.keys():
                    frontier[neighbor] = tentative_score
                elif tentative_score >= frontier[neighbor]:
                    continue
                came_from[neighbor] = current_node
                g_score[neighbor] = g_score[current_node] + self.uniform_cost
            self.visualize_step(visited, frontier, current_node)

    def dijkstra_plan(self):
        # visited
        visited = []
        # frontier with g score
        frontier = {self.start: 0}
        # running cost for node
        g_score = {self.start: 0}
        # back track
        came_from = {}
        while len(frontier) > 0:
            print("current frontier length: ", len(frontier))
            current_node = min(frontier, key=frontier.get)
            if current_node == self.goal:
                print("path found")
                return self.reconstruct_path(came_from, current_node)

            current_node_value = frontier.pop(current_node)
            visited.append(current_node)

            neighbors = self.get_neighbors(current_node)
            for neighbor in neighbors:
                if neighbor in visited:
                    continue
                tentative_score = g_score[current_node] + self.uniform_cost
                if neighbor not in frontier.keys():
                    frontier[neighbor] = tentative_score
                elif tentative_score >= frontier[neighbor]:
                    continue
                came_from[neighbor] = current_node
                g_score[neighbor] = g_score[current_node] + self.uniform_cost
            self.visualize_step(visited, frontier, current_node)

    def reconstruct_path(self, came_from, current_node):
        wp_list = [current_node]
        while current_node in came_from.keys():
            wp_list.insert(0, came_from[current_node])
            current_node = came_from[current_node]
        return wp_list

    def get_neighbors(self, current_node):
        neighbors = []
        # 8 connected
        for r in range(-1, 2):
            for c in range(-1, 2):
                n_r = current_node[0]+r
                n_c = current_node[1]+c
                if n_r < 0 or n_r >= self.map_height or n_c < 0 or n_c >= self.map_width:
                    continue
                if self.occgrid[n_r, n_c] == 0:
                    neighbors.append((n_r, n_c))
                else:
                    continue
        # print("neighbors: ", neighbors)
        return neighbors

    def visualize_step(self, visited, frontier, current_node):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.CUBE_LIST
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04

        visited_col = ColorRGBA()
        visited_col.a = 1.0
        visited_col.r = 0.8
        visited_col.g = 0.1
        visited_col.b = 0.1

        frontier_col = ColorRGBA()
        frontier_col.a = 1.0
        frontier_col.r = 0.0
        frontier_col.g = 0.8
        frontier_col.b = 0.1

        current_col = ColorRGBA()
        current_col.a = 1.0
        current_col.r = 0.0
        current_col.g = 0.0
        current_col.b = 1.0

        # visualize visted
        for node in visited:
            cube = self.rc_2_coord(node[0], node[1])
            marker.points.append(cube)
            marker.colors.append(visited_col)
        # visualize frotier
        for node in frontier.keys():
            cube = self.rc_2_coord(node[0], node[1])
            marker.points.append(cube)
            marker.colors.append(frontier_col)
        # visualize current node
        cube = self.rc_2_coord(current_node[0], current_node[1])
        marker.points.append(cube)
        marker.colors.append(current_col)
        
        self.viz_pub.publish(marker)

    def visualize_path(self, path):
        # visualize path
        path_msg = Path()
        path_msg.header.frame_id = "/map"
        for node in path:
            node_pose = PoseStamped()
            cube = self.rc_2_coord(node[0], node[1])
            node_pose.pose.position.x = cube.x
            node_pose.pose.position.y = cube.y
            path_msg.poses.append(node_pose)
        self.path_pub.publish(path_msg)

    def start_callback(self, start_msg):
        start_x = start_msg.pose.pose.position.x
        start_y = start_msg.pose.pose.position.y
        start_r, start_c = self.coord_2_rc(start_x, start_y)
        if self.start == (start_r, start_c):
            return
        else:
            self.start = (start_r, start_c)
        
        if self.goal == None:
            return
        else:
            if self.use_astar:
                self.path = self.astar_plan()
            else:
                self.path = self.dijkstra_plan()
        # TODO decide what to do next
        self.visualize_path(self.path)

    def goal_callback(self, goal_msg):
        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y
        goal_r, goal_c = self.coord_2_rc(goal_x, goal_y)

        if self.goal == (goal_r, goal_c):
            return
        else:
            self.goal = (goal_r, goal_c)
        
        if self.start == None:
            return
        else:
            if self.use_astar:
                self.path = self.astar_plan()
            else:
                self.path = self.dijkstra_plan()
        # TODO: decide what to do next
        self.visualize_path(self.path)


def main():
    rospy.init_node('path_planner_toy')
    planner = PathPlanner()
    rospy.spin()

if __name__ == '__main__':
    main()