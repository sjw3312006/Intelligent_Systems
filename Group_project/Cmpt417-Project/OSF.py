# -*- coding: utf-8 -*-
"""
Created on Mon Apr 19 11:40:32 2021

@author: wlian
"""
import numpy as np
import math 

def h_manhattan(goal, index, val):
    size = len(goal)
    n = int(math.sqrt(size))
    goal_indx = goal.index(val)
    goal_x = goal_indx // n
    goal_y = goal_indx - n * goal_x
    # print(goal_indx, goal_x, goal_y)

    index_x = index // n
    index_y = index - n * index_x
    
    # print(index, index_x, index_y)
    dist = (abs(index_x - goal_x) + abs(index_y - goal_y))
    return dist

def disp(size, osf):
    for i in range(size):
        for j in range(size-1):
            print("i:", i, "j:", j, end=" -> ")
            for k in range(4):
                print(osf[i][j][k], end=", ")
            print()

def cal_osf(n, goal):
    size = n
    n = int(math.sqrt(size))
    osf = np.zeros(shape=(size,size-1,4))

    actions = [-n, n, -1, 1]
    for i in range(size):
        for j in range(size-1):
            val = j + 1
            for k in range(len(actions)):
                # up
                if k == 0 and i < n:
                    osf[i][j][k] = math.inf
                    continue
                # down
                if k == 1 and i > n*n-n-1:
                    osf[i][j][k] = math.inf
                    continue
                # left
                if k == 2 and i % n == 0:
                    osf[i][j][k] = math.inf
                    continue
                # right
                if k == 3 and i % n == n-1:
                    osf[i][j][k] = math.inf
                    continue
                    
                new_dist = h_manhattan(goal, i, val)
                old_dist = h_manhattan(goal, i + actions[k], val)
                osf[i][j][k] = 1 + (new_dist - old_dist)
    return osf

# goal =  list(range(1,9))
# goal.append(0)
# osf = cal_osf(9,goal)
# print('h:',h_manhattan(goal, 8, 8))










