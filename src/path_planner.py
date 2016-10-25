#!/usr/bin/env python

import roslib

import rospy

from std_msgs.msg import Header, Empty, Float32, Float32MultiArray, String, Int32, Header
from sensor_msgs.msg import Image, CompressedImage, ChannelFloat32, PointCloud
from geometry_msgs.msg import Point, Quaternion, Point32, PointStamped, PoseStamped                                     #http://docs.ros.org/api/geometry_msgs/html/index-msg.html

from tf import transformations

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

from target import Target

from constant import *
from vector import *
from angle import *

from scipy import spatial
import Queue

class TREE(object):
    def __init__(self, root):
        self.root = root

        # tree edges related: used for traverse
        self.bottomup = {self.root : (None, None)}  # map from child to (parent, edge_cost)
        self.topdown = {self.root : []}             # map from parent to children

        # tree nodes related: used for nearest neighbor query
        self.node_costs = {self.root : 0}           # map from node to cost

        self.kd_tree = None
        self.unorganized_nodes = [self.root]
        self.size_unorganized_nodes = 1
        self.max_size_unorganized_nodes = 512

    def add_edge(self, parent, child, cost):
        self.bottomup[child] = (parent, cost)
        if not self.topdown.has_key(parent):
            self.topdown[parent] = [child]
        else:
            self.topdown[parent] = self.topdown[parent] + [child]

        self.node_costs[child] = self.node_costs[parent] + cost

        if self.node_costs.has_key(child):
            return

        self.unorganized_nodes = self.unorganized_nodes + [child]
        self.size_unorganized_nodes += 1
        if self.size_unorganized_nodes >= self.max_size_unorganized_nodes:
            all_nodes = map(tuple, self.kd_tree.data) + self.unorganized_nodes
            self.kd_tree = spatial.KDTree(all_nodes)
            self.unorganized_nodes = []
            self.size_unorganized_nodes = 0

    def rewire_node(self, new_parent, child, new_edge_cost, new_total_cost):
        old_parent, old_edge_cost = self.bottomup[child]
        old_total_cost = self.node_costs[child]
        reduced_cost = old_total_cost - new_total_cost

        self.bottomup[child] = (new_parent, new_edge_cost)
        self.topdown[old_parent].remove(child)

        queue = Queue.Queue()
        queue.put(child)
        while not queue.empty():
            top = queue.get()
            self.node_costs -= reduced_cost
            if self.topdown.has_key(top):
                for ch in self.topdown[top]:
                    queue.put(ch)

    def nearest_neighbor(self, node):
        dist, neighbor = self.distance_between(self.root, node), self.root
        for neighbor_in_list in self.unorganized_nodes:
            dist_in_list = self.distance_between(neighbor_in_list, node)
            if dist_in_list < dist:
                dist, neighbor = dist_in_list, neighbor_in_list

        if len(self.kd_tree.data) > 0:
            dist_in_tree, neighbor_idx_in_tree = self.kd_tree.query(node)
            if dist_in_tree < dist:
                dist, neighbor = dist_in_tree, tuple(self.kd_tree.data[neighbor_idx_in_tree])
        return neighbor

    def neighbors_in_ball(self, node, radius=None, beta=10):
        if radius is None:
            radius = beta * math.log(len(self.node_costs.keys())) / len(self.node_costs.keys())
        neighbors = []
        for neighbor_in_list in self.unorganized_nodes:
            dist_in_list = self.distance_between(neighbor_in_list, node)
            if dist_in_list < radius:
                neighbors += [neighbor_in_list]

        if len(self.kd_tree.data) > 0:
            neighbors += map(lambda i: tuple(self.kd_tree.data[i]), self.kd_tree.query_ball_point(node, radius))

        return neighbors

    def distance_between(self, n1, n2):
        return ((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2 + (n1[2] - n2[2])**2)**0.5


class RRT_STAR_SOLVER(object):
    # map_points in format of [(x,y,z)...] in world cooridinate
    def __init__(self, current_position, goal_position, map_points):
        self.start = current_position
        self.goal = goal_position # (x,y,z) in world coordinate

        #  search tree
        self.tree = TREE(self.start)

        # obstacles
        self.map_points_kd_tree = spatial.KDTree(map_points) # [(x,y,z)] in world coordinate

        # workspace boundary
        x_s = map(lambda pt: pt[0], map_points) + [self.start[0], self.goal[0], -1, 1]
        y_s = map(lambda pt: pt[1], map_points) + [self.start[1], self.goal[2], -1, 1]
        z_s = map(lambda pt: pt[2], map_points) + [self.start[2], self.goal[2], -1, 1]
        min_x = min(x_s)
        max_x = max(x_s)
        min_y = min(y_s)
        max_y = max(y_s)
        min_z = min(z_s)
        max_z = max(z_s)
        self.boundaries = [(2 * min_x - max_x, 2 * max_x - min_x) ,(2 * min_y - max_y, 2 * max_y - min_y),(2 * min_z - max_z, 2 * max_z - min_z)]

    def get_trace_to_goal(self):
        for loop in range(NUM_NODES_RRT_STAR):
            node_random = self.random_sample()
            if (loop+1) % 20 == 0:
                node_random = self.goal

            node_nearest = self.tree.nearest_neighbor(node_random)
            node_new = self.steer_end(node_nearest, node_random)
            if self.is_collision(node_new):
                continue

            nodes_nearby = self.tree.neighbors_in_ball(node_new)

            edges_cost = dict() # one end of the edge is node_new, the other end is a node in nodes_nearby, which is mapped to edge cost
            for n in nodes_nearby:
                edges_cost[n] = self.steer_cost_simple(node_new, n)

            parent, edge_cost, total_cost = node_nearest, self.steer_cost_simple(node_new, node_nearest), self.tree.node_costs[node_nearest] + self.steer_cost_simple(node_new, node_nearest)
            for n in nodes_nearby:
                total_cost_in_list = self.tree.node_costs[n] + edges_cost[n]
                if total_cost_in_list < total_cost:
                    parent, edge_cost, total_cost = n, edges_cost[n], total_cost_in_list

            self.tree.add_edge(parent, node_new, edge_cost)

            for n in nodes_nearby:
                if total_cost + edges_cost[n] < self.tree.node_costs[n]:
                    self.tree.rewire_node(node_new, n, edges_cost[n], total_cost + edges_cost[n])

        # back track
        # goal_nodes = self.tree.neighbors_in_ball(self.goal,radius=0.1)
        # if len(goal_nodes) <= 0:
        #     return None
        if not self.tree.node_costs.has_key(self.goal):
            return None
        trace = [self.goal]
        while self.tree.bottomup[trace[-1]][0] is not None:
            trace += [self.tree.bottomup[trace[-1]][0]]
        return trace

    def distance_between(self, n1, n2):
        return ((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2 + (n1[2] - n2[2])**2)**0.5

    def utility(self, node):
        near_by_map_points = self.map_points_kd_tree.query_ball_point(node, ALPHA * 3)
        # print near_by_map_points
        utility = 0
        for idx in near_by_map_points:
            utility += max(0, min(COLLISION_PENALTY, COLLISION_PENALTY *(1 - self.distance_between(node, self.map_points_kd_tree.data[idx]) / (3 * ALPHA))))
        return utility

    def is_collision(self, node):
        return self.utility(node) > 2. / 3. * COLLISION_PENALTY

    def random_sample(self):
        x_random = np.random.uniform(self.boundaries[0][0], self.boundaries[0][1])
        y_random = np.random.uniform(self.boundaries[1][0], self.boundaries[1][1])
        z_random = np.random.uniform(self.boundaries[2][0], self.boundaries[2][1])

        return (x_random, y_random, z_random)

    def steer_end(self, start, goal, max_step = 0.1):
        dist = self.distance_between(start, goal)
        if dist <= max_step:
            return goal
        else:
            return (start[0] + (goal[0] - start[0]) / dist * max_step,
                    start[1] + (goal[1] - start[1]) / dist * max_step,
                    start[2] + (goal[2] - start[2]) / dist * max_step)


    def steer_cost_hard(self, start, goal):
        dist = self.distance_between(start, goal)
        num_steps = math.ceil(dist / STEP_LENGTH)

        step_len = dist / num_steps
        step = ((goal[0] - start[0]) / num_steps, (goal[1] - start[1]) / num_steps, (goal[2] - start[2]) / num_steps)

        total_cost = 0
        for i in range(num_steps):
            internal_node = (start[0] + step[0] * (i+1), start[1] + step[1] * (i+1), start[2] + step[2] * (i+1))
            total_cost += self.utility(internal_node) * step_len

        return total_cost / dist

    def steer_cost_simple(self, start, goal):
        return (self.utility(start) + self.utility(goal)) * self.distance_between(start, goal) / 2

