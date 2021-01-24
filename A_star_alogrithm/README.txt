The values used for testing while coding the astar.py file are:

1) forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east

This was used to check the manuever with respect to the 2D A star algorithm without the involment of orientation.

2) action name = ['R', 'F', 'L'] 

To check the direction during manuevering and appended it to the path.

3) cost = [1, 1, 1]

To add the value for predicting the g value. 

4) grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

The grid structure for the path to move. Here 0 represent the space available for navigation and 1 represent the unnavigable space.

5) starting position or the node = (4, 3, 0)

6) ending position or goal node = (2, 0, 1) #incase of 3D A star algorithm.

7) heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]

This heuristic matrix helps in giving the h value for calculating the estimated cost to reach the goal node.

8) Assigned the value of g to be zero and it increases with the decrease in h value to attain the f value.

The values in the nodes are put and popped out when needed like node and child in the code.

The libraries used here is import (Value, OrderedSet, PriorityQueue).

For the dijkstra algorithm, we used the import heapq as hq library.
