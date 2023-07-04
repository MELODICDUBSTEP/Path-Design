import DR20API
import numpy as np
import heapq as hq
import math
from queue import PriorityQueue

### START CODE HERE ###
# This code block is optional. You can define your utility function and class in this block if necessary.

MAX_A_STAR_NUM = 20

COLS = ROWS = 120

PUNISH_1 = 2
PUNISH_2 = 1.2
PUNISH_3 = 1.08
PUNISH_4 = 1.015


F = 0.8

Staright_reward1 = 0.83
Staright_reward2 = 0.96

class node:
    def __init__(self, f, g, h):
        self.f = f
        self.g = g
        self.h = h
        self.p_x = -1
        self.p_y = -1
        self.l = 0
        self.p = 1
        self.move = ''

def H(x, y, goal_pos):
    return math.sqrt((x - goal_pos[0]) * (x - goal_pos[0]) + (y - goal_pos[1]) * (y - goal_pos[1]))

def l1(x1, y1, x2, y2):
    return max(abs(x1 - x2), abs(y1 - y2))

def setpunish(current_map, nodes):
    for i in range(0, ROWS):
        for j in range(0, COLS):
            if current_map[i][j] == 1:
                direction = [-3, -2, -1, 0, 1, 2, 3]
                for s in direction:
                    for t in direction:
                        if i + s >= 0 and i + s < 120 and j + t >= 0 and j + t < 120:
                            if l1(i, j, i + s, j + t) == 1:
                                nodes[i + s][j + t].p *= PUNISH_1
                            elif l1(i, j, i + s, j + t) == 2:
                                nodes[i + s][j + t].p *= PUNISH_2
                            elif l1(i, j, i + s, j + t) == 3:
                                nodes[i + s][j + t].p *= PUNISH_3
                            elif l1(i, j, i + s, j + t) == 4:
                                nodes[i + s][j + t].p *= PUNISH_4
    

###  END CODE HERE  ###

def A_star(current_map, current_pos, goal_pos):
    """
    Given current map of the world, current position of the robot and the position of the goal, 
    plan a path from current position to the goal using A* algorithm.

    Arguments:
    current_map -- A 120*120 array indicating current map, where 0 indicating traversable and 1 indicating obstacles.
    current_pos -- A 2D vector indicating the current position of the robot.
    goal_pos -- A 2D vector indicating the position of the goal.

    Return:
    path -- A N*2 array representing the planned path by A* algorithm.
    """

    ### START CODE HERE ###
    
    closedList = [[0 for i in range(COLS)] for j in range(ROWS)]
    nodes = [[node(200000.0, 100000.0, 100000.0) for i in range(COLS)] for j in range(ROWS)]
    setpunish(current_map, nodes)
    nodes[current_pos[0]][current_pos[1]].f = 0.0
    nodes[current_pos[0]][current_pos[1]].g = 0.0
    nodes[current_pos[0]][current_pos[1]].h = 0.0
    nodes[current_pos[0]][current_pos[1]].l = 1
    #print("!")
    # We initialize the 2-D array for A*
    
    pq = [(0.0, [current_pos[0], current_pos[1]])]
    hq.heapify(pq)
    x = 0
    y = 0
    
    while pq:
        temp = hq.heappop(pq)
        x = temp[1][0]
        y = temp[1][1]
        if closedList[x][y] == 1:
            continue
        #print(x, y)
        closedList[x][y] = 1
        
        if x - 1 >= 0 and x - 1 < 120:
            if x - 1 == goal_pos[0] and y == goal_pos[1]:
                nodes[x - 1][y].p_x = x
                nodes[x - 1][y].p_y = y
                break
            elif(closedList[x - 1][y] == 0 and current_map[x - 1][y] == 0):
                #print('!')
                g1 = nodes[x][y].g + 1.0
                h1 = H(x - 1, y, goal_pos)
                f1 = (g1 + h1 * F) * nodes[x - 1][y].p
                if nodes[x][y].move == 'left':
                    f1 *= Staright_reward1
                elif nodes[x][y].move == 'up_left' or nodes[x][y] == 'down_left':
                    f1 *= Staright_reward2
                #elif nodes[x][y].move == 'right':
                 #   f1 *= S_punish
                if(nodes[x - 1][y].f > f1):
                    hq.heappush(pq, (f1, [x - 1, y]))
                    nodes[x - 1][y].f = f1
                    nodes[x - 1][y].g = g1
                    nodes[x - 1][y].h = h1
                    nodes[x - 1][y].p_x = x
                    nodes[x - 1][y].p_y = y
                    nodes[x - 1][y].l = nodes[x][y].l + 1
                    nodes[x - 1][y].move = 'left'
                    if(nodes[x][y].l + 1 >= 10):
                        break
                    
        if x + 1 >= 0 and x + 1 < 120:
            if x + 1 == goal_pos[0] and y == goal_pos[1]:
                nodes[x + 1][y].p_x = x
                nodes[x + 1][y].p_y = y
                break
            elif(closedList[x + 1][y] == 0 and current_map[x + 1][y] == 0):
                g1 = nodes[x][y].g + 1.0
                h1 = H(x + 1, y, goal_pos)
                f1 = (g1 + h1 * F) * nodes[x + 1][y].p
                if nodes[x][y].move == 'right':
                    f1 *= Staright_reward1
                elif nodes[x][y].move == 'up_right' or nodes[x][y] == 'down_right':
                    f1 *= Staright_reward2
                if(nodes[x + 1][y].f > f1):
                    hq.heappush(pq, (f1, [x + 1, y]))
                    nodes[x + 1][y].f = f1
                    nodes[x + 1][y].g = g1
                    nodes[x + 1][y].h = h1
                    nodes[x + 1][y].p_x = x
                    nodes[x + 1][y].p_y = y
                    nodes[x + 1][y].l = nodes[x][y].l + 1
                    nodes[x + 1][y].move = 'right'
                    if(nodes[x][y].l + 1 >= 10):
                        break
                    
        if y + 1 >= 0 and y + 1 < 120:
            if x == goal_pos[0] and y + 1 == goal_pos[1]:
                nodes[x][y + 1].p_x = x
                nodes[x][y + 1].p_y = y
                break
            elif(closedList[x][y + 1] == 0 and current_map[x][y + 1] == 0):
                g1 = nodes[x][y].g + 1.0
                h1 = H(x, y + 1, goal_pos)
                f1 = (g1 + h1 * F) * nodes[x][y + 1].p
                if nodes[x][y].move == 'up':
                    f1 *= Staright_reward1
                elif nodes[x][y].move == 'up_right' or nodes[x][y] == 'up_left':
                    f1 *= Staright_reward2
                if(nodes[x][y + 1].f > f1):
                    hq.heappush(pq, (f1, [x, y + 1]))
                    nodes[x][y + 1].f = f1
                    nodes[x][y + 1].g = g1
                    nodes[x][y + 1].h = h1
                    nodes[x][y + 1].p_x = x
                    nodes[x][y + 1].p_y = y
                    nodes[x][y + 1].l = nodes[x][y].l + 1
                    nodes[x][y + 1].move = 'up'
                    if(nodes[x][y].l + 1 >= 10):
                        break
                    
        if y - 1 >= 0 and y - 1 < 120:
            if x == goal_pos[0] and y - 1 == goal_pos[1]:
                nodes[x][y - 1].p_x = x
                nodes[x][y - 1].p_y = y
                break
            elif(closedList[x][y - 1] == 0 and current_map[x][y - 1] == 0):
                g1 = nodes[x][y].g + 1.0
                h1 = H(x, y - 1, goal_pos)
                f1 = (g1 + h1 * F) * nodes[x][y - 1].p
                if nodes[x][y].move == 'down':
                    f1 *= Staright_reward1
                elif nodes[x][y].move == 'down_right' or nodes[x][y] == 'down_left':
                    f1 *= Staright_reward2
                if(nodes[x][y - 1].f > f1):
                    hq.heappush(pq, (f1, [x, y - 1]))
                    nodes[x][y - 1].f = f1
                    nodes[x][y - 1].g = g1
                    nodes[x][y - 1].h = h1
                    nodes[x][y - 1].p_x = x
                    nodes[x][y - 1].p_y = y
                    nodes[x][y - 1].l = nodes[x][y].l + 1
                    nodes[x][y - 1].move = 'down'
                    if(nodes[x][y].l + 1 >= 10):
                        break
                    
            if x + 1 >= 0 and x + 1 < 120 and y + 1 >= 0 and y + 1 < 120:
                if x + 1 == goal_pos[0] and y + 1 == goal_pos[1]:
                    nodes[x + 1][y + 1].p_x = x
                    nodes[x + 1][y + 1].p_y = y
                    break
                elif(closedList[x + 1][y + 1] == 0 and current_map[x + 1][y + 1] == 0):
                    g1 = nodes[x][y].g + 1.4
                    h1 = H(x + 1, y + 1, goal_pos)
                    f1 = (g1 + h1 * F) * nodes[x + 1][y + 1].p
                    if nodes[x][y].move == 'up_right':
                        f1 *= Staright_reward1
                    elif nodes[x][y].move == 'right' or nodes[x][y] == 'up':
                        f1 *= Staright_reward2
                    if(nodes[x + 1][y + 1].f > f1):
                        hq.heappush(pq, (f1, [x + 1, y + 1]))
                        nodes[x + 1][y + 1].f = f1
                        nodes[x + 1][y + 1].g = g1
                        nodes[x + 1][y + 1].h = h1
                        nodes[x + 1][y + 1].p_x = x
                        nodes[x + 1][y + 1].p_y = y
                        nodes[x + 1][y + 1].l = nodes[x][y].l + 1
                        nodes[x + 1][y + 1].move = 'up_right'
                        if(nodes[x][y].l + 1 >= 10):
                            break
            if x + 1 >= 0 and x + 1 < 120 and y - 1 >= 0 and y - 1 < 120:
                if x + 1 == goal_pos[0] and y - 1 == goal_pos[1]:
                    nodes[x + 1][y - 1].p_x = x
                    nodes[x + 1][y - 1].p_y = y
                    break
                elif(closedList[x + 1][y - 1] == 0 and current_map[x + 1][y - 1] == 0):
                    g1 = nodes[x][y].g + 1.4
                    h1 = H(x + 1, y - 1, goal_pos)
                    f1 = (g1 + h1 * F) * nodes[x + 1][y - 1].p
                    if nodes[x][y].move == 'down_right':
                        f1 *= Staright_reward1
                    elif nodes[x][y].move == 'down' or nodes[x][y] == 'right':
                        f1 *= Staright_reward2
                    if(nodes[x + 1][y - 1].f > f1):
                        hq.heappush(pq, (f1, [x + 1, y - 1]))
                        nodes[x + 1][y - 1].f = f1
                        nodes[x + 1][y - 1].g = g1
                        nodes[x + 1][y - 1].h = h1
                        nodes[x + 1][y - 1].p_x = x
                        nodes[x + 1][y - 1].p_y = y
                        nodes[x + 1][y - 1].l = nodes[x][y].l + 1
                        nodes[x + 1][y - 1].move = 'down_right'
                        if(nodes[x][y].l + 1 >= 10):
                            break
            if x - 1 >= 0 and x - 1 < 120 and y + 1 >= 0 and y + 1 < 120:
                if x - 1 == goal_pos[0] and y + 1 == goal_pos[1]:
                    nodes[x - 1][y + 1].p_x = x
                    nodes[x - 1][y + 1].p_y = y
                    break
                elif(closedList[x - 1][y + 1] == 0 and current_map[x - 1][y + 1] == 0):
                    g1 = nodes[x][y].g + 1.4
                    h1 = H(x - 1, y + 1, goal_pos)
                    f1 = (g1 + h1 * F) * nodes[x - 1][y + 1].p
                    if nodes[x][y].move == 'up_left':
                        f1 *= Staright_reward1
                    elif nodes[x][y].move == 'up' or nodes[x][y] == 'left':
                        f1 *= Staright_reward2
                    if(nodes[x - 1][y + 1].f > f1):
                        hq.heappush(pq, (f1, [x - 1, y + 1]))
                        nodes[x - 1][y + 1].f = f1
                        nodes[x - 1][y + 1].g = g1
                        nodes[x - 1][y + 1].h = h1
                        nodes[x - 1][y + 1].p_x = x
                        nodes[x - 1][y + 1].p_y = y
                        nodes[x - 1][y + 1].l = nodes[x][y].l + 1
                        nodes[x - 1][y + 1].move = 'up_left'
                        if(nodes[x][y].l + 1 >= 10):
                            break
                    
            if x - 1 >= 0 and y - 1 < 120:
                if x - 1 == goal_pos[0] and y - 1 == goal_pos[1]:
                    nodes[x - 1][y - 1].p_x = x
                    nodes[x - 1][y - 1].p_y = y
                    break
                elif(closedList[x - 1][y - 1] == 0 and current_map[x - 1][y - 1] == 0):
                    g1 = nodes[x][y].g + 1.4
                    h1 = H(x - 1, y - 1, goal_pos)
                    f1 = (g1 + h1 * F) * nodes[x - 1][y - 1].p
                    if nodes[x][y].move == 'down_left':
                        f1 *= Staright_reward1
                    elif nodes[x][y].move == 'down' or nodes[x][y] == 'left':
                        f1 *= Staright_reward2
                    if(nodes[x - 1][y - 1].f > f1):
                        hq.heappush(pq, (f1, [x - 1, y - 1]))
                        nodes[x - 1][y - 1].f = f1
                        nodes[x - 1][y - 1].g = g1
                        nodes[x - 1][y - 1].h = h1
                        nodes[x - 1][y - 1].p_x = x
                        nodes[x - 1][y - 1].p_y = y
                        nodes[x - 1][y - 1].l = nodes[x][y].l + 1
                        nodes[x - 1][y - 1].move = 'down_left'
                        if(nodes[x][y].l + 1 >= 10):
                            break
                        
    
    path = []
    while nodes[x][y].p_x != -1:
        path.append([x, y])
        x = nodes[x][y].p_x
        y = nodes[x][y].p_y
    path.reverse()
    print(path)
    return path
    ###  END CODE HERE  ###

def reach_goal(current_pos, goal_pos):
    """
    Given current position of the robot, 
    check whether the robot has reached the goal.

    Arguments:
    current_pos -- A 2D vector indicating the current position of the robot.
    goal_pos -- A 2D vector indicating the position of the goal.

    Return:
    is_reached -- A bool variable indicating whether the robot has reached the goal, where True indicating reached.
    """

    ### START CODE HERE ###
    is_reached = (current_pos[0] == goal_pos[0]) and (current_pos[1] == goal_pos[1])

    ###  END CODE HERE  ###
    return is_reached

if __name__ == '__main__':
    # Define goal position of the exploration, shown as the gray block in the scene.
    goal_pos = [100, 100]
    controller = DR20API.Controller()

    # Initialize the position of the robot and the map of the world.
    current_pos = controller.get_robot_pos()
    current_map = controller.update_map()

    # Plan-Move-Perceive-Update-Replan loop until the robot reaches the goal.
    while not reach_goal(current_pos, goal_pos):
        # Plan a path based on current map from current position of the robot to the goal.
        path = A_star(current_map, current_pos, goal_pos)
        # Move the robot along the path to a certain distance.
        controller.move_robot(path) 
        # Get current position of the robot.
        current_pos = controller.get_robot_pos()
        # Update the map based on the current information of laser scanner and get the updated map.
        current_map = controller.update_map()

    # Stop the simulation.
    controller.stop_simulation()