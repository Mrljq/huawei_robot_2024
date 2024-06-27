# 这里记录算法的实现，寻路之后传回path_calculate规划路径
# -*- coding: utf-8 -*-
"""

@author: gaohaotian
"""
import math

from model import *
import numpy as np
from collections import deque




def get_obstacles(the_map):
    obstacles = []
    for i in range(len(the_map)):
        for j in range(len(the_map)):
            if the_map[i][j] == '#' or the_map[i][j] =='*':  # 如果为‘#’，则为障碍物
                x = i 
                y = j
                obstacle = (x, y)
                obstacles.append(obstacle)
    
    return  obstacles

#从方向表中提取路径。输入方向表格：200*200的numpy，目标位置（x，y）
def extract_path(direction_table, target_position):
    target_x, target_y = target_position
    path = [(target_x, target_y)]

    import sys
    while direction_table[target_x][target_y] != 0:
        # print((target_x, target_y),file=sys.stderr)
        
        direction = direction_table[target_x][target_y]
        if direction == 1:
            target_x += 1
        elif direction == 2:
            target_x -= 1
        elif direction == 3:
            target_y += 1
        elif direction == 4:
            target_y -= 1

        path.insert(0, (target_x, target_y))

    return path



def is_valid(x, y, rows, cols):
    return 0 <= x < rows and 0 <= y < cols



#查找港口到所有点的路径：
'''
initial_position:港口坐标
obstacle_coordinates:障碍物列表
rows,cols:地图大小
return: paths该港口到所有位置的方向表格 paths_len各个位置到港口的路径长度
'''
def find_paths(initial_position, obstacle_coordinates, rows, cols):
    obstacles = set(obstacle_coordinates)
    paths = np.zeros((rows, cols), dtype=int) -1  # Initialize with -1 to represent unvisited cells
    paths_len=np.zeros((rows, cols), dtype=int) +99999
    start_x, start_y = initial_position
    
    visited = set()
    queue = deque([(start_x, start_y)])

    paths[start_x][start_y] = 0  # Distance to the initial position is 0
    paths_len[start_x][start_y] = 0
    while queue:
        current_x, current_y = queue.popleft()

        if (current_x, current_y) not in visited:
            visited.add((current_x, current_y))

            for dx, dy, direction in [(-1, 0, 1), (1, 0, 2), (0, -1, 3), (0, 1, 4)]:
                new_x, new_y = current_x + dx, current_y + dy

                if is_valid(new_x, new_y, rows, cols) and (new_x, new_y) not in visited and (new_x, new_y) not in obstacles:
                    paths[new_x][new_y] = direction
                    paths_len[new_x][new_y] = paths_len[current_x][current_y]+1
                    queue.append((new_x, new_y))

    return paths,paths_len




###########################AAAAAAAA*###########################################

# A*算法
def astar(start, goal, obstacles):
    open_set = set()
    closed_set = set()
    current = start
    open_set.add(current)
    # counter = 1
    while open_set:
        # counter += 1
        # if counter >= 10000:
        #     return None
        current = min(open_set, key=lambda x: x.g + x.h)
        open_set.remove(current)
        closed_set.add(current)

        if get_distance(current, goal) ==0:
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1] 

        for neighbor in get_neighbors(current, obstacles):
            if neighbor in closed_set:
                continue
            # tentative_g = current.g+ 1/3*get_distance(current, neighbor)
            tentative_g =  get_distance(current, neighbor)
            if neighbor not in open_set:
                open_set.add(neighbor)
                neighbor.h = get_distance(neighbor, goal)
            elif tentative_g >= neighbor.g:
                continue

            neighbor.parent = current
            neighbor.g = tentative_g

    return None


# 获取当前点的邻居点
def get_neighbors(node, obstacles):
    neighbors = []
    for x in range(-1, 2):
        for y in range(-1, 2):
            if x == 0 and y == 0 or(x*y!=0):
                continue
            if node.x + x <0 or node.x + x >200 or node.y + y <0 or node.y + y > 200 or Node(
                    node.x + x , node.y + y ) in obstacles:
                continue
            neighbor = Node(node.x + x , node.y + y )
            neighbors.append(neighbor)
    return neighbors
 

def get_distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)


def get_manhadundis(node1, node2):
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)


