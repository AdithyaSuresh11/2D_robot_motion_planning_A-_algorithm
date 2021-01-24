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

###################### Manuevering action with respect to parent and child ##################
def rotate(node1,node):
    action = node[2] - node1[2]
    if action == -3:
        return 1
    if action == 3:
        return -1
    else:
        return action

#################### Change in orientation for the current position ###############
def change_angle(node0,node):
    direction = [node[0] - node0[0],node[1] - node0[1]]
    if direction == forward[0]:
        return 0
    if direction == forward[1]:
        return 1
    if direction == forward[2]:
        return 2
    if direction == forward[3]:
        return 3

#################### Function for finding the children ####################
def children_nodes(grid,node0):
    children_need = []
    children = [(node0[0],node0[1]-1,0), (node0[0]-1, node0[1],0), (node0[0], node0[1] + 1,0),(node0[0]+ 1, node0[1],0)]
    for node in children:
        val_x=node[0]
        val_y=node[1]
        if (len(grid)>val_x>= 0 and len(grid[0])>val_y>= 0):
            if (grid[val_x][val_y] == 0):
                direction_need = change_angle(node0, node)
                #print('////////////////////////////////////////////////')
                #print('The direction is:', direction_need)
                node = [val_x,val_y,direction_need]
                if rotate(node0,node) != -2 and rotate(node0,node) != 2:
                    children_need.append((val_x,val_y,direction_need))
    return children_need

################### Function for estimated cost ##################
def heuristics_value(cost_new,action,pose):
    h_value = heuristic[pose[0]][pose[1]]
    g_value = cost_new.g + 1
    f = g_value + h_value + cost[action+1]
    return f, g_value

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

    # Implement of A* algorithm:

    while open_set:
        node1,cost_new = open_set.pop()
        print('The current node is:', node1)
        print('---------------------------------')
        if node1 == goal:
            closed_set.add(node1)
            while node1 != start:
                node0 = parent[node1[2]][node1[0]][node1[1]]
                action = rotate(node0,node1)
                node1 = node0
                path[node0[0]][node0[1]] = action_name[action+1]
            return path,closed_set
        children = children_nodes(grid, node1)
        print('The children present are:', children)
        print('==============================================================================')
        closed_set.add(node1)

        for child in children:
            direction = rotate(node1,child)
            cost_child, child_new = heuristics_value(cost_new,direction,child)
            if child not in open_set or closed_set:
                open_set.put(child,Value(f=cost_child,g=child_new))

            if child in open_set:
                cost_need = open_set.get(child)
                cost_f = cost_need.f
                if cost_child < cost_f:
                    open_set.put(child,Value(f=cost_child, g=child_new))

            if parent_cost[child[2]][child[0]][child[1]] > cost_child:
               parent_cost[child[2]][child[0]][child[1]] = cost_child
               parent[child[2]][child[0]][child[1]] = node1[0],node1[1],node1[2]
    return path, closed_set

if __name__ == "__main__":
    path,closed=compute_path(grid, init, goal, cost, heuristic)

    for i in range(len(path)):
        print(path[i])

    print("\nExpanded Nodes:")
    for node in closed:
        print(node)

"""
To test the correctness of your A* implementation, when using cost = [1, 1, 10] your code should return

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-']

In this case, the elements in your closed set (i.e. the expanded nodes) are:
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

"""
