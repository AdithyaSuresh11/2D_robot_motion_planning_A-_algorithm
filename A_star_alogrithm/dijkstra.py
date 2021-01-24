# Compute the optimal path from start to goal.
# The car is moving on a 2D grid and
# its orientation can be chosen from four different directions:
forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east

# The car can perform 3 direction: -1: right turn and then move forward, 0: move forward, 1: left turn and then move forward
action = [-1, 0, 1] ###Action performed for manuevering
action_name = ['R', 'F', 'L']   ###Action name associated for manuevering
cost = [1, 1, 10] # corresponding cost val10es

grid = [[1, 1, 1, 0, 0, 0],     ###Path grid predefined
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = (4, 3, 0) # (grid row, grid col, orientation)

goal = (2, 0, 1) # (grid row, grid col, orientation)

#goal = (2,0,0) # (grid row, grid col, orientation)

heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
            [1, 2, 3, 4, 5, 6],
            [0, 1, 2, 3, 4, 5],
            [1, 2, 3, 4, 5, 6],
            [2, 3, 4, 5, 6, 7]]

from utils import (Value, OrderedSet, PriorityQueue)
import heapq as hq
inf = float('inf')

def compute_path(grid,start,goal,cost,heuristic):
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)

    parent = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]

    # The path of the car
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]

    x = start[0]
    y = start[1]
    theta = start[2]
    g_value = 0
    h_value = heuristic[x][y]
    f = g_value + h_value
    parent_cost = [[[ 1000000000 for row in range(len(grid[0]))] for col in range(len(grid))],
                  [[ 1000000000 for row in range(len(grid[0]))] for col in range(len(grid))],
                  [[ 1000000000 for row in range(len(grid[0]))] for col in range(len(grid))],
                  [[ 1000000000 for row in range(len(grid[0]))] for col in range(len(grid))]]
    open_set.put(start, Value(f=f,g=g_value))

########################### IMPLEMENTATION OF DIJKSTRA ALGORITHM ###################################
####################################################################################################
####################################################################################################
    def dijkstra(grid, source):
        length= len(grid)
        Q_value = [(0,source)]
        value0 = [inf for i in range(length)]
        value0[source] = 0

        while len(Q) != 0:
            (cost, u) = hq.heappop(Q_value)

            for v in range(n):
                if value0[v] > value0[u] + grid[u][v]:
                    value0[v] = value0[u] + grid[u][v]
                    hq.heappush(Q, (value0[v], v))

        return value0
        print('The dijkstra value is:', value0)
    return path, closed_set
####################################################################################################
####################################################################################################
####################################################################################################

if __name__ == "__main__":
    path,closed=compute_path(grid, init, goal, cost, heuristic)

    for i in range(len(path)):
        print(path[i])

    print("\nExpanded Nodes:")
    for node in closed:
        print(node)
