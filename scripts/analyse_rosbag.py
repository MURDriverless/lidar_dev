'''
Author: Steven Lee

Python script for inspecting recorded rosbag.
It is assumed that all relevant LiDAR topics are recorded.

References:
https://answers.ros.org/question/55037/is-there-a-way-to-save-all-rosbag-data-into-a-csv-or-text-file/

Usage:
    python analyse_rosbag.py
    (remember to update the parameters below)
'''

import rosbag
from mur_common.msg import cone_msg
from mur_common.msg import timing_msg

import math
import matplotlib.pyplot as plt
import os
import csv
import numpy as np
import pandas as pd
import pickle

BLUE = 'Blue'
YELLOW = 'Yellow'

# ground truth info
# x_truth = [1.7, 1.8, 1.9, 2.1]
# y_truth = [-0.1, -0.4, 0.2, 0.5]
# c_truth = ['Blue', 'Blue', 'Yellow', 'Yellow']

# ground truth for 2m 4m all
# x_truth = [2, 4, 2, 4]
# y_truth = [-1.5, -1.5, 1.5, 1.5]
# c_truth = ['Blue', 'Blue', 'Yellow', 'Yellow']

x_truth = [3, 5, 3, 5]
y_truth = [-1.5, -1.5, 1.5, 1.5]
c_truth = ['Blue', 'Blue', 'Yellow', 'Yellow']

class analysis:
    def __init__(self)]:
        # open rosbag from specified location
        # self.bag = rosbag.Bag(os.path.expanduser('~/bagfiles/lidar_bench_test_all_topics.bag'))
        self.bag = rosbag.Bag(os.path.expanduser('~/bagfiles/os1_3m_5m_all.bag'))
        self.topic = '/cluster_node/cone_messages'
        self.column_names = ['x', 'y', 'colour']
        self.scatter_marker_sz = 50

    def parse_ground_truths(self):
        """
        Parse ground truth measurements
        x_truth: x positions
        y_truth: y positions
        c_truth: colours
        """

        x_blue_truth = []
        y_blue_truth = []
        x_yellow_truth = []
        y_yellow_truth = []

        for idx, val in enumerate(x_truth):
            if c_truth[idx] == BLUE:
                x_blue_truth.append(x_truth[idx])
                y_blue_truth.append(y_truth[idx])
            elif c_truth[idx] == YELLOW:
                x_yellow_truth.append(x_truth[idx])
                y_yellow_truth.append(y_truth[idx])

        # convert lists to np arrays
        x_blue_truth    = np.asarray(x_blue_truth)
        y_blue_truth    = np.asarray(y_blue_truth)
        x_yellow_truth  = np.asarray(x_yellow_truth)
        y_yellow_truth  = np.asarray(y_yellow_truth)

        self.x_blue_truth = x_blue_truth  
        self.y_blue_truth = y_blue_truth  
        self.x_yellow_truth = x_yellow_truth
        self.y_yellow_truth = y_yellow_truth
    
    def parse_estimated_results(self):
        """
        For plotting a single instant of the rosbag
        """

        for topic, msg, t in self.bag.read_messages(topics=self.topic):
            # rotate around z by 180 deg 
            x = -np.asarray(msg.x)
            y = -np.asarray(msg.y)
            colour = msg.colour

            # lists to store estimated cone position
            x_blue = []
            y_blue = []
            x_yellow = []
            y_yellow = []

            for idx, val in enumerate(x):
                if colour[idx] == BLUE:
                    x_blue.append(x[idx])
                    y_blue.append(y[idx])
                elif colour[idx] == YELLOW:
                    x_yellow.append(x[idx])
                    y_yellow.append(y[idx])

            # convert lists to np arrays
            self.x_blue = np.asarray(x_blue)
            self.y_blue = np.asarray(y_blue)
            self.x_yellow = np.asarray(x_yellow)
            self.y_yellow = np.asarray(y_yellow)
            break

    def update_estimated_results(self, msg_x, msg_y, msg_c):
        x = -np.asarray(msg_x)
        y = -np.asarray(msg_y)
        colour = msg_c

        # lists to store estimated cone position
        x_blue = []
        y_blue = []
        x_yellow = []
        y_yellow = []

        for idx, val in enumerate(x):
            if colour[idx] == BLUE:
                x_blue.append(x[idx])
                y_blue.append(y[idx])
            elif colour[idx] == YELLOW:
                x_yellow.append(x[idx])
                y_yellow.append(y[idx])

        # convert lists to np arrays
        self.x_blue = np.asarray(x_blue)
        self.y_blue = np.asarray(y_blue)
        self.x_yellow = np.asarray(x_yellow)
        self.y_yellow = np.asarray(y_yellow)

    def compute_precision_recall(self):
        """
        Compute precision and recall for LiDAR intensity image classifier
        """

        self.parse_ground_truths()
        n_total = 0
        n_correct = 0

        TP = 0
        TN = 0
        FP = 0
        FN = 0
        
        # lets try to benchmark for the blue cone first
        for topic, msg, t in self.bag.read_messages(topics=self.topic):
            x = -np.asarray(msg.x)
            y = -np.asarray(msg.y)
            c = msg.colour

            for idx_t, val_t in enumerate(c_truth):
                # first lets locate the corresponding ground truth
                min_dist = float('inf')
                est_index = -1
                for idx, val in enumerate(c):
                    dist = self.euclidean_dist( (x[idx], y[idx]), (x_truth[idx_t], y_truth[idx_t]) )
                    if dist < min_dist:
                        min_dist = dist
                        est_index = idx

                # check estimated colour with ground truth colour
                n_total += 1
                if min_dist < 0.5:
                    # ! debug print
                    # print(c[est_index], c_truth[idx_t])
                    # blue, yellow

                    if c[est_index] == c_truth[idx_t]:
                        n_correct += 1
                    if c[est_index] == BLUE and c_truth[idx_t] == BLUE:
                        TP += 1
                    if c[est_index] == BLUE and c_truth[idx_t] == YELLOW:
                        FP += 1
                    if c[est_index] == YELLOW and c_truth[idx_t] == YELLOW:
                        TN += 1
                    if c[est_index] == YELLOW and c_truth[idx_t] == BLUE:
                        FN += 1
        
        # print results
        accuracy = float(n_correct) / n_total
        precision = float(TP) / (TP + FP)
        recall = float(TP) / (TP + FN)

        print(n_correct)
        print(n_total)
        print('TP = ', TP)
        print('FP = ', FP)
        print('TN = ', TN)
        print('FN = ', FN)
        print("LiDAR classifier accuracy = ", accuracy)
        print("Precision = ", precision)
        print("Recall = ", recall)

    def compute_precision_recall_yellow(self):
        """
        Compute precision and recall for LiDAR intensity image classifier
        This is for the yellow cone class.
        """

        self.parse_ground_truths()
        n_total = 0
        n_correct = 0

        TP = 0
        TN = 0
        FP = 0
        FN = 0
        
        # lets try to benchmark for the blue cone first
        for topic, msg, t in self.bag.read_messages(topics=self.topic):
            x = -np.asarray(msg.x)
            y = -np.asarray(msg.y)
            c = msg.colour

            for idx_t, val_t in enumerate(c_truth):
                # first lets locate the corresponding ground truth
                min_dist = float('inf')
                est_index = -1
                for idx, val in enumerate(c):
                    dist = self.euclidean_dist( (x[idx], y[idx]), (x_truth[idx_t], y_truth[idx_t]) )
                    if dist < min_dist:
                        min_dist = dist
                        est_index = idx

                # check estimated colour with ground truth colour
                n_total += 1
                if min_dist < 0.5:
                    # ! debug print
                    # print(c[est_index], c_truth[idx_t])
                    # blue, yellow

                    if c[est_index] == c_truth[idx_t]:
                        n_correct += 1

                    if c[est_index] == YELLOW and c_truth[idx_t] == YELLOW:
                        TP += 1
                    if c[est_index] == YELLOW and c_truth[idx_t] == BLUE:
                        FP += 1
                    if c[est_index] == BLUE and c_truth[idx_t] == BLUE:
                        TN += 1
                    if c[est_index] == BLUE and c_truth[idx_t] == YELLOW:
                        FN += 1
        
        # print results
        accuracy = float(n_correct) / n_total
        precision = float(TP) / (TP + FP)
        recall = float(TP) / (TP + FN)

        print(n_correct)
        print(n_total)
        print('TP = ', TP)
        print('FP = ', FP)
        print('TN = ', TN)
        print('FN = ', FN)
        print("LiDAR classifier accuracy = ", accuracy)
        print("Precision = ", precision)
        print("Recall = ", recall)

    def plot_single_instant(self):
        """
        Generates a PDF plot for an instant of the rosbag
        """
        self.parse_ground_truths()
        self.parse_estimated_results()

        # plot lidar sensor position
        plt.scatter(0, 0, color='black', label="LiDAR Position", s=self.scatter_marker_sz)

        # plot estimated positions
        plt.scatter(self.x_blue, self.y_blue, s=self.scatter_marker_sz, marker='x', c='b', label='Estimated Cone Pose')
        plt.scatter(self.x_yellow, self.y_yellow, s=self.scatter_marker_sz, marker='x', c='y')

        # plot ground truth positions
        plt.scatter(self.x_blue_truth, self.y_blue_truth, s=self.scatter_marker_sz, c='none', edgecolors='b', label='True Cone Pose')
        plt.scatter(self.x_yellow_truth, self.y_yellow_truth, s=self.scatter_marker_sz, c='none', edgecolors='y')

        axes = plt.gca()
        fig = plt.gcf()

        plt.xlim(-1, 5)
        plt.ylim(-3, 3)

        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        # plt.axis('square')

        axes.set_aspect('equal', adjustable='box')
        axes.legend()
        axes.grid(True)

        plt.savefig('results.pdf')
        plt.show()
        plt.savefig('results_2.pdf')

    def compute_rmse(self):
        '''
        Computes RMSE by matching estimated landmark with closest ground
        truth
        '''
        pass

    def euclidean_dist(self, tuple1, tuple2):
        '''
        Computes euclidean distance between two tuples
        (x1, y1) and (x2, y2)
        '''
        (x1, y1) = tuple1
        (x2, y2) = tuple2
        return math.sqrt( (y2-y1)**2 + (x2-x1)**2 )

if __name__ == "__main__":
    analyse = analysis()
    # analyse.plot_single_instant()
    analyse.compute_precision_recall()
    print('************************************************************')
    analyse.compute_precision_recall_yellow()