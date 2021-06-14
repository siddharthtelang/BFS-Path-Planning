#!/usr/bin/python3

# ========================================
# ENPM661 Spring 2021: Planning for Autonomous Robots
# Project 2
# Point robot planning in an obstacle environment using BFS algorithm
#
# Author: Siddharth Telang(stelang@umd.edu)
# ========================================

import numpy as np
import matplotlib.pyplot as plt
import math
import cv2

# define a Queue class for storing the nodes to be expanded
class Queue:

    # init function
    def __init__(self):
        self.queue = []

    # insert in queue
    def enque(self, item):
        self.queue.insert(0, item)

    # pop node
    def deque(self):
        if self.queue:
            return self.queue.pop()
        return 'Empty'

    # check if queue is not empty
    def not_empty(self):
        return (len(self.queue) > 0)

    # check if the queue contains a specific list item
    def contains(self, item):
        return (item in self.queue)

    # print the queue
    def __print__(self):
        print(self.queue)

# define a class Node for storing all nodes generated in map with parent node info
class Node:

    # init function to store info about:
    # child: own state and move
    # parent: index, state and move
    def __init__(self, state, parent_index, move):
        self.state = state # child list
        self.move = move
        self.parent_index = parent_index # parent index
        self.parent_state = [] # parent list/state form
        self.parent_move = ''
