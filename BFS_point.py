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

# function to check if the move is in obstacle space
def isInObstacleSpace(i,j):

    if (i > 399 or i < 0 or j < 0 or j > 299):
        print('Tending out of boundary ; avoid')
        return 1

    #condition for cicle
    circle = (i-90)**2 + (j-70)**2
    if circle <= 1225:
        print('Tending towards circle ; avoid')
        return 1

    #condition for ellipse
    ellipse = ((i-246)**2)/(60*60) + ((j-145)**2)/(30*30)
    if ellipse <= 1.0:
        print('Tending towards ellipse ; avoid')
        return 1

    #condition for rectangle
    d1 = abs((j - 0.7002*i - 74.39) / (1 + (0.7002)**2)**(0.5))
    d2 = abs((j - 0.7002*i - 98.8) / (1 + (0.7002)**2)**(0.5))
    d3 = abs((j + 1.428*i - 176.55) / (1 + (1.428)**2)**(0.5))
    d4 = abs((j + 1.428*i - 439.44) / (1 + (1.428)**2)**(0.5))
    if (d1+d2 <= 22.0 and d3+d4 <= 152.0):
        print('Tending towards rectangle ; avoid')
        return 1

    # condition for quadilaterl
    d1 = abs((j-i+265) / (2**0.5))
    d2 = abs((j-i+180) / (2**0.5))
    d3 = abs((j+i-497) / (2**0.5))
    d4 = abs((j+i-391) / (2**0.5))
    d5 = abs((j-i+210) / (2**0.5))
    d6 = abs((j+i-497) / (2**0.5))
    d7 = abs((j+i-552) / (2**0.5))
    d8 = abs((j+(0.1135*i)-182) / (1 + (0.1135**2))**0.5)

    if ((d1<=1.5 and j>=63 and j<=116) or (d4<=1.5 and j>=63 and j<=105 ) or (d2 <= 1.5 and j>=105 and j<=145) or
        (d8<=1.5 and j>=142 and j<=145 and i>=325 and i<=352) or (d5<=1.5 and j>=142 and j<=171) or (i==381 and j<=171 and j>=116)):
        print('Tending towards quad; avoid')
        return 1

    # condition for C shaped obstacle
    if ((abs(i-200) <= 0 and (j >= 230 and j <= 280)) or
            ((abs(j-230) <=0 or abs(j-280) <=0) and (i >= 200 and i <= 230)) or
                (abs(i-210) <= 0 and (j >= 240 and j <= 270)) or
                    (abs(i-230) <= 0 and ((j >= 230 and j <= 240) or (j >= 270 and j <= 280))) or
                        ((abs(j-270) <= 0 or abs(j-240) <= 0) and (i >= 210 and i <= 230))):
        print('Tending towards C type object ; avoid')
        return 1

    return 0

# function to find the moves from given coordinates
def find_moves(i,j, obj):
    moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
    final_moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
    move_i = [i, i+1, i+1, i+1, i, i-1, i-1, i-1]
    move_j = [j+1, j+1, j, j-1, j-1, j-1, j, j+1]
    for move in range(len(moves)):
        if (isInObstacleSpace(move_i[move], move_j[move]) or obj.parent_state == [move_i[move], move_j[move]]):
            final_moves.remove(moves[move])
    #print(final_moves)
    return final_moves


def printDebugLines(mainList, backTrack):
    print(mainList)
    print('---------------------------------------------------------------------\n\n')
    for i in range(len(backTrack)):
        print('--------------------------------------------------------------------')
        print(backTrack[i].state)
        print(backTrack[i].parent_index)
        print(backTrack[i].parent_state)
        print('---------------------------------------------------------------------\n')

# function to back track the nodes
def backTrace(backTrack, goal):
    track, parent_state, last_parent_index  = [], [], -1
    for i in reversed(range(len(backTrack))):
        if last_parent_index == backTrack[i].parent_index:
            continue
        if backTrack[i].state == goal or backTrack[i].state == parent_state:
            last_parent_index = backTrack[i].parent_index
            parent_state = backTrack[i].parent_state
            track.insert(0,backTrack[i])
    track.pop(0)
    return track

def generateMap(xlist, ylist, mainList):
    map = cv2.imread('map.jpg')
    map = cv2.resize(map, (401,301))
    # describe the boundary with red lines
    map[:1,:], map[:,:1], map[:,-2], map[-1,:] = [0,0,255], [0,0,255], [0,0,255], [0,0,255]
    video = cv2.VideoWriter('BFS_Final_Video.avi',cv2.VideoWriter_fourcc(*'XVID'), 100,(401,301))

    # for all the visited nodes, set the region as white
    for i in range(len(mainList)):
        # change to image coordinate for Y axis
        (mainList[i])[1] = 300 -  (mainList[i])[1]
        x = (mainList[i])[0]
        y = (mainList[i])[1]
        map[y][x] = 255
        video.write(map)

    # wait for 2s
    for i in range(0,200):
        video.write(map)

    # highlight the final path from initial to goal state in green color
    for i,j in zip(xlist,ylist):
        map[j][i] = [0,100,0]
        video.write(map)

    # wait for 2s before the video finishes
    for i in range(0,200):
        video.write(map)

    video.release()
    map = cv2.resize(map, (1080,720))
    # save the final map having the highlighted path and traversed nodes
    cv2.imwrite('final_map.jpg', map)

