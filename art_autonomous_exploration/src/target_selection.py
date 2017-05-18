#!/usr/bin/env python

import rospy
import random
import math
import time
import numpy as np
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import OgmOperations
from utilities import Print
from brushfires import Brushfires
from topology import Topology
import scipy
from path_planning import PathPlanning
from robot_perception import RobotPerception


# Class for selecting the next best target
class TargetSelection:
    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.path = []
        self.prev_target = [0, 0]
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method

        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()
        self.robot_perception = RobotPerception()  # can i use that?

    def selectTarget(self, init_ogm, coverage, robot_pose, origin, \
                     resolution, force_random=False):

        target = [-1, -1]
        print resolution

        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the 
        # following tools: ogm_limits, Brushfire field, OGM skeleton,
        # topological nodes.

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                                                 origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                                           resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)
        print len(nodes)

        # Visualization of topological nodes
        vis_nodes = []
        for n in nodes:
            vis_nodes.append([
                n[0] * resolution + origin['x'],
                n[1] * resolution + origin['y']
            ])
        RvizHandler.printMarker( \
            vis_nodes, \
            1,  # Type: Arrow
            0,  # Action: Add
            "map",  # Frame
            "art_topological_nodes",  # Namespace
            [0.3, 0.4, 0.7, 0.5],  # Color RGBA
            0.1  # Scale
        )

        # Random point
        # if self.method == 'random' or force_random == True:
        #     target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
        #
        # if self.method == 'distance':
        #     target = self.selectNearestTopologyNode(robot_pose, resolution, nodes)
        target = self.selectNearestTopologyNode(robot_pose=robot_pose, resolution=resolution, nodes=nodes, ogm=ogm,
                                                coverage=coverage, brushogm=brush)
        ########################################################################

        return target

    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
        # The next target in pixels
        tinit = time.time()
        next_target = [0, 0]
        found = False
        while not found:
            x_rand = random.randint(0, ogm.shape[0] - 1)
            y_rand = random.randint(0, ogm.shape[1] - 1)
            if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and \
                            brushogm[x_rand][y_rand] > 5:
                next_target = [x_rand, y_rand]
                found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit),
                        Print.ORANGE)
        return next_target

    # way too slow, maybe i didn't quite understood weigthed path find?
    def selectNearestTopologyNode(self, robot_pose, resolution, nodes, ogm, coverage, brushogm):
        # The next target in pixels
        tinit = time.time()
        next_target = [0, 0]
        w_dist = 10000
        x_g, y_g = 500, 500
        sigma = 100
        g_robot_pose = self.robot_perception.getGlobalCoordinates(
            [robot_pose['x_px'],
             robot_pose['y_px']])  # can I use that?
        for node in nodes:
            # print resolution
            self.path = self.path_planning.createPath(
                g_robot_pose,
                node,
                resolution)  # can I use that?
            if len(self.path)==0:
                break
            x_n, y_n = node[0], node[1]
            exp = ((x_n - x_g) ** 2 + (y_n - y_g) ** 2) / (2 * (sigma ** 2))
            scale_factor = 1 / (1 - math.exp(-exp) + 0.01)

            # dist = np.asarray(self.path).transpose()  # dist is [[x1 x2 x3..],[y1 y2 y3 ..]]
            dist = zip(*self.path)  # dist is [[x1 x2 x3..],[y1 y2 y3 ..]] #transpose

            shift = np.empty_like(dist)
            shift[0] = dist[0][-1:] + dist[0][:-1] #(xpi+1)
            shift[1] = dist[1][-1:] + dist[1][:-1] #(ypi+1)

            dist = np.asarray(dist)
            shift = np.asarray(shift)
            dist = [(a - b)**2 for a, b in zip(dist[:, 1:], shift[:, 1:])]  # (xpi - xpi+1)^2 , (ypi-ypi+1)^2
            dist = sum(np.sqrt(dist[0]+dist[1]))

            # w = len(self.path) * (1 / scale_factor)  # na ginei swsta to W giati to len path exei ta stoixeia oxi tis apostaseis
            w = dist * (1 / scale_factor)  # na ginei swsta to W giati to len path exei ta stoixeia oxi tis apostaseis
            print (self.path)
            if w < w_dist and len(self.path) != 0:
                if self.prev_target == node:
                    break
                w_dist = w
                # if ogm[x_n][y_n] < 50 and coverage[x_n][y_n] < 50 and \
                #                 brushogm[x_n][y_n] > 5:
                #     print "mpika"
                print len(self.path)

                next_target = node

        self.prev_target = next_target

        Print.art_print("Select nearest node target time: " + str(time.time() - tinit),
                        Print.ORANGE)
        print next_target
        return next_target
