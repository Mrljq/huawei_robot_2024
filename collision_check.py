#问题简化成了对于两个相邻距离为1的机器人，如何防止两个机器人碰撞；
#继续简化，如果两个机器人第i步可能会发生冲突，如果对方的位置没有出现在第i+1步中，我认为可以让一个机器人等待，另一个机器人前往，这样用一个指令时间去等待；
#如果

# map1 26w 195535
# map2 221897 223190 219289 22w 174837
# map3 249293 248980 249679 25w 191453 靠墙的碰撞
# map4 19w 143657 港口 单通道都有问题
# map5 271914 269837 272304 27w 194757
# map6 数据未知 267511
# map7 273455 273556 258200 27w 222281
# map8 270200 267036 266544 27w 208391 出现在港口的碰撞 三个机器人
import numpy as np
from model import *
from util import *
import random
import time
import itertools
direction = [[1,0],[0,1],[-1,0],[0,-1]]


def detect_collision(id ,paths , bots , track_index):
    
    bot_num = len(bots)
    collision_flag = True
    reback_status = [0] * 10
    now_position = set()
    robot_next_pos = [] # 存储机器人的位置信息
    # nums = []

    #初始化robot_pos
    for index in range(bot_num):
        now_position.add(Node(bots[index].x , bots[index].y))
        if track_index[index] < len(paths[index]) and isinstance(paths[index][track_index[index]] , tuple) and len(paths[index][track_index[index]]) > 1:
            robot_next_pos.append(Node(paths[index][track_index[index]][0],paths[index][track_index[index]][0]))
        else:
            robot_next_pos.append(Node(-1,-1))

    while(collision_flag):
        collision_flag = False
        for robot_1 in range(bot_num):
            for robot_2 in range(robot_1+1 , bot_num):
                if len(paths[robot_1]) <= 1 or len(paths[robot_2]) <= 1 or track_index[robot_1] < 0 or track_index[robot_2] < 0 or track_index[robot_1] >= len(paths[robot_1]) or track_index[robot_2] >= len(paths[robot_2]):
                    continue
                
                # 两种情况，直接冲突和交换冲突
                # if (paths[robot_1][track_index[robot_1]][0] == paths[robot_2][track_index[robot_2]][0] and paths[robot_1][track_index[robot_1]][1] == paths[robot_2][track_index[robot_2]][1]) or ((bots[robot_1].x == paths[robot_2][track_index[robot_2]][0] and bots[robot_1].y == paths[robot_2][track_index[robot_2]][1]) and (bots[robot_2].x == paths[robot_1][track_index[robot_1]][0] and bots[robot_2].y == paths[robot_1][track_index[robot_1]][1])):
                if paths[robot_1][track_index[robot_1]] == paths[robot_2][track_index[robot_2]] or ((bots[robot_1].x , bots[robot_1].y) == paths[robot_2][track_index[robot_2]] and (bots[robot_2].x , bots[robot_2].y) == paths[robot_1][track_index[robot_1]]):   
                    # nums.append([robot_1,robot_2])
                    # 如果某个机器人已经发生过冲突，那么只对未发生冲突的机器人或者发生冲突次数少的机器人进行处理
                    timestamp = int(time.time())
                    random.seed(a=timestamp)
                    chosen_number = random.choice([1,2])
                    target_robot = robot_2
                    if chosen_number == 1:
                        target_robot = robot_2
                        if reback_status[robot_1] < reback_status[robot_2]:
                            target_robot = robot_1
                    else:
                        # 第二种方案，进行随机的选取目标机器人
                        target_robot = random.choice([robot_1 , robot_2])

                    #编号小的保持不变，编号大的进行处理，即robot_2，这里规定0代表未发生碰撞，1代表停止，2代表其他两个方向寻路，==3代表回退
                    reback_status[target_robot] = reback_status[target_robot] + 1
                    if reback_status[target_robot] >= 4:# 已经进行过所有避障操作
                        continue
                    elif reback_status[target_robot] == 3: # 已经产生了左右退的现象，弹出插入的点执行回退
                        del paths[target_robot][track_index[target_robot]]
                        track_index[target_robot] = max(0 , track_index[target_robot]-1)
                    elif reback_status[target_robot] == 1 and paths[robot_1][track_index[robot_1]] == paths[robot_2][track_index[robot_2]]: #产生相撞于一点
                        track_index[target_robot] = max(0 , track_index[target_robot]-1)
                    elif (reback_status[target_robot] == 1 and ((bots[robot_1].x , bots[robot_1].y) == paths[robot_2][track_index[robot_2]] and (bots[robot_2].x , bots[robot_2].y) == paths[robot_1][track_index[robot_1]])):
                        left_right_flag = False # 判断其他两个方向有没有空地
                        wait_pos = [] #构建可选点，长度为1或者2
                        for dict in direction:
                            pos = (bots[target_robot].x+dict[0] , bots[target_robot].y+dict[1])
                            pos_1 = Node(bots[target_robot].x+dict[0],bots[target_robot].y+dict[1])
                            if min(pos[0],pos[1]) >= 0 and max(pos[0] , pos[1]) <= 200 and pos != paths[target_robot][track_index[target_robot]] and pos_1 not in obstacles_set and pos_1 not in robot_next_pos and pos_1 not in now_position: # 测试时修改为live_points_1，同时传入
                                if track_index[target_robot] >= 2 and pos == paths[target_robot][track_index[target_robot]-2]:
                                    continue
                                # print(target_robot , track_index[target_robot] , pos.x , pos.y , bots[target_robot].x , bots[target_robot].y)
                                wait_pos.append(pos)
                                left_right_flag = True
                        if not left_right_flag: # 没有左右的执行回退
                            track_index[target_robot] = max(0 , track_index[target_robot]-2)
                            reback_status[target_robot] = 3
                        else:
                            pos = wait_pos[0]
                            if len(wait_pos) == 2:
                                pos = random.choice(wait_pos)
                            paths[target_robot].insert(track_index[target_robot] , pos)
                            paths[target_robot].insert(track_index[target_robot]+1 , (bots[target_robot].x , bots[target_robot].y))
                            reback_status[target_robot] = 2 # 发生过左右的情况
                    elif reback_status[target_robot] == 2:# 这里是对之前相撞于一点的情况进行处理
                        left_right_flag = False # 判断其他两个方向有没有空地
                        wait_pos = [] #构建可选点，长度为1或者2
                        for dict in direction:
                            pos = (bots[target_robot].x+dict[0] , bots[target_robot].y+dict[1])
                            pos_1 = Node(bots[target_robot].x+dict[0] , bots[target_robot].y+dict[1])
                            if min(pos[0],pos[1]) >= 0 and max(pos[0] , pos[1]) <= 200 and pos != paths[target_robot][track_index[target_robot]+1] and pos not in obstacles and pos_1 not in robot_next_pos and pos_1 not in now_position: # 测试时修改为live_points_1，同时传入
                                if track_index[target_robot] >= 1 and pos == paths[target_robot][track_index[target_robot]-1]:
                                    continue
                                # print(target_robot , track_index[target_robot] , pos.x , pos.y , bots[target_robot].x , bots[target_robot].y)
                                wait_pos.append(pos)
                                left_right_flag = True
                        if not left_right_flag: # 没有左右的执行回退
                            track_index[target_robot] = max(0 , track_index[target_robot]-1)
                            reback_status[target_robot] = 3
                        else:
                            pos = wait_pos[0]
                            if len(wait_pos) == 2:
                                pos = random.choice(wait_pos)
                            paths[target_robot].insert(track_index[target_robot] , pos)
                            reback_status[target_robot] = 2 # 发生过左右的情况
                    collision_flag = True

                    # # 更新robot_next_pos
                    if isinstance(paths[index][track_index[index]] , tuple) and len(paths[target_robot][track_index[target_robot]]) > 1:
                        robot_next_pos[target_robot] = Node(paths[target_robot][track_index[target_robot]][0] , paths[target_robot][track_index[target_robot]][0])
    # print(track_index[0],track_index[1],track_index[2],track_index[3],track_index[4],track_index[5],track_index[6],track_index[7],track_index[8],track_index[9],file=sys.stderr)
    # 检测计算得到的下一个前往的点是否交叉
    # for i in range(10):
    #     for j in range(10):
    #         if i == j:
    #             continue
    #         if track_index[i] < len(paths[i]) and track_index[j] < len(paths[j]) and ((paths[i][track_index[i]] == (bots[j].x , bots[j].y) and paths[j][track_index[j]] == (bots[i].x , bots[i].y)) or (paths[i][track_index[i]] == paths[j][track_index[j]])):
    #             print(id, file=sys.stderr)
    #             print(paths[i], file=sys.stderr)
    #             print(paths[j], file=sys.stderr)     
    #             print(i,j,paths[i][track_index[i]],paths[j][track_index[j]], file=sys.stderr)
    #             print(id,file=sys.stderr)
    #             print(reback_status,file=sys.stderr)
    #             print(nums,file=sys.stderr)
    # 计算是否能执行
    # for i in range(10):
    #     if track_index[i] < len(paths[i]) and isinstance(paths[i][track_index[i]] , tuple) and len(paths[i][track_index[i]]) == 2:
    #         length = abs(bots[i].x-paths[i][track_index[i]][0]) + abs(bots[i].y-paths[i][track_index[i]][1])
    #         if length > 1:
    #             print(id,i,bots[i].x, bots[i].y, paths[i][track_index[i]], bots[i].status,file=sys.stderr)
    # print(track_index[0],track_index[1],track_index[2],track_index[3],track_index[4],track_index[5],track_index[6],track_index[7],track_index[8],track_index[9],file=sys.stderr)
    # print(len(paths[0]),len(paths[1]),len(paths[2]),len(paths[3]),len(paths[4]),len(paths[5]),len(paths[6]),len(paths[7]),len(paths[8]),len(paths[9]),file=sys.stderr)
    return track_index,paths


def detect_collision_1(paths , robots , track_index):
    
    bot_num = 10
    collision_flag = True
    reback_status = [0] * 10
    now_position = set()
    robot_next_pos = [] # 存储机器人的位置信息
    bots = list(itertools.combinations(robots, 2))
    waits = [[] for _ in range(10)]
    random.shuffle(bots)
    reback_step = 5
    # nums = []

    #初始化robot_pos
    for index in range(bot_num):
        now_position.add(Node(robots[index].x , robots[index].y))
        if track_index[index] < len(paths[index]) and isinstance(paths[index][track_index[index]] , tuple) and len(paths[index][track_index[index]]) > 1:
            robot_next_pos.append(Node(paths[index][track_index[index]][0],paths[index][track_index[index]][0]))
        else:
            robot_next_pos.append(Node(-1,-1))

    while(collision_flag):
        collision_flag = False
        for bot in bots:
            robot_1 = bot[0].id
            robot_2 = bot[1].id
            if len(paths[robot_1]) <= 1 or len(paths[robot_2]) <= 1 or track_index[robot_1] < 0 or track_index[robot_2] < 0 or track_index[robot_1] >= len(paths[robot_1]) or track_index[robot_2] >= len(paths[robot_2]):
                continue
            
            # 两种情况，直接冲突和交换冲突
            # if (paths[robot_1][track_index[robot_1]][0] == paths[robot_2][track_index[robot_2]][0] and paths[robot_1][track_index[robot_1]][1] == paths[robot_2][track_index[robot_2]][1]) or ((bots[robot_1].x == paths[robot_2][track_index[robot_2]][0] and bots[robot_1].y == paths[robot_2][track_index[robot_2]][1]) and (bots[robot_2].x == paths[robot_1][track_index[robot_1]][0] and bots[robot_2].y == paths[robot_1][track_index[robot_1]][1])):
            if paths[robot_1][track_index[robot_1]] == paths[robot_2][track_index[robot_2]] or ((robots[robot_1].x , robots[robot_1].y) == paths[robot_2][track_index[robot_2]] and (robots[robot_2].x , robots[robot_2].y) == paths[robot_1][track_index[robot_1]]):   
                # nums.append([robot_1,robot_2])
                # 如果某个机器人已经发生过冲突，那么只对未发生冲突的机器人或者发生冲突次数少的机器人进行处理
                # timestamp = int(time.time())
                # random.seed(a=timestamp)
                # chosen_number = random.choice([1,2])
                # target_robot = robot_2
                # if chosen_number == 1:
                target_robot = robot_2
                if reback_status[robot_1] < reback_status[robot_2]:
                    target_robot = robot_1
                # else:
                #     # 第二种方案，进行随机的选取目标机器人
                #     target_robot = random.choice([robot_1 , robot_2])

                #编号小的保持不变，编号大的进行处理，即robot_2，这里规定0代表未发生碰撞，1代表停止，2代表其他两个方向寻路，==3代表回退
                reback_status[target_robot] = reback_status[target_robot] + 1
                if reback_status[target_robot] >= 4:# 已经进行过所有避障操作
                    continue
                elif reback_status[target_robot] == 3: # 已经产生了左右退的现象，弹出插入的点执行回退
                    if len(waits[target_robot]) == 2: # 如果还有备选位置，执行另一个躲避位置
                        paths[target_robot][track_index[target_robot]] = waits[target_robot][1]
                        waits[target_robot] = []
                        reback_status[target_robot] = 2
                    else:
                        del paths[target_robot][track_index[target_robot]]
                        track_index[target_robot] = max(0 , track_index[target_robot]-1)
                        # 在路径中插入多个回退
                        # if track_index[target_robot] > 0:
                        #     tmp_step = min(reback_step , track_index[target_robot])
                        #     next_step = []
                        #     for i in range(track_index[target_robot]-1 , track_index[target_robot]-1-tmp_step , -1):
                        #         next_step.append(paths[target_robot][i])
                        #     for i in range(track_index[target_robot]-tmp_step+1 , track_index[target_robot]+1):
                        #         next_step.append(paths[target_robot][i])
                        #     # 将list插入到路径中
                        #     paths[target_robot].insert(track_index[target_robot]+1 , next_step)
                elif reback_status[target_robot] == 1 and paths[robot_1][track_index[robot_1]] == paths[robot_2][track_index[robot_2]]: #产生相撞于一点
                    track_index[target_robot] = max(0 , track_index[target_robot]-1)
                elif (reback_status[target_robot] == 1 and ((robots[robot_1].x , robots[robot_1].y) == paths[robot_2][track_index[robot_2]] and (robots[robot_2].x , robots[robot_2].y) == paths[robot_1][track_index[robot_1]])):
                    left_right_flag = False # 判断其他两个方向有没有空地
                    wait_pos = [] #构建可选点，长度为1或者2
                    for dict in direction:
                        pos = (robots[target_robot].x+dict[0] , robots[target_robot].y+dict[1])
                        pos_1 = Node(robots[target_robot].x+dict[0],robots[target_robot].y+dict[1])
                        if min(pos[0],pos[1]) >= 0 and max(pos[0] , pos[1]) <= 200 and pos != paths[target_robot][track_index[target_robot]] and pos_1 not in obstacles_set and pos_1 not in robot_next_pos and pos_1 not in now_position: # 测试时修改为live_points_1，同时传入
                            if track_index[target_robot] >= 2 and pos == paths[target_robot][track_index[target_robot]-2]:
                                continue
                            # print(target_robot , track_index[target_robot] , pos.x , pos.y , bots[target_robot].x , bots[target_robot].y)
                            wait_pos.append(pos)
                            left_right_flag = True
                    if not left_right_flag: # 没有左右的执行回退
                        track_index[target_robot] = max(0 , track_index[target_robot]-2)
                        reback_status[target_robot] = 3
                        # if track_index[target_robot] > 0:
                        #     tmp_step = min(reback_step , track_index[target_robot])
                        #     next_step = []
                        #     for i in range(track_index[target_robot]-1 , track_index[target_robot]-1-tmp_step , -1):
                        #         next_step.append(paths[target_robot][i])
                        #     for i in range(track_index[target_robot]-tmp_step+1 , track_index[target_robot]+1):
                        #         next_step.append(paths[target_robot][i])
                        #     # 将list插入到路径中
                        #     paths[target_robot].insert(track_index[target_robot]+1 , next_step)
                    else:
                        pos = wait_pos[0]
                        waits[target_robot] = wait_pos
                        paths[target_robot].insert(track_index[target_robot] , pos)
                        paths[target_robot].insert(track_index[target_robot]+1 , (robots[target_robot].x , robots[target_robot].y))
                        reback_status[target_robot] = 2 # 发生过左右的情况
                elif reback_status[target_robot] == 2:# 这里是对之前相撞于一点的情况进行处理
                    left_right_flag = False # 判断其他两个方向有没有空地
                    wait_pos = [] #构建可选点，长度为1或者2
                    for dict in direction:
                        pos = (robots[target_robot].x+dict[0] , robots[target_robot].y+dict[1])
                        pos_1 = Node(robots[target_robot].x+dict[0] , robots[target_robot].y+dict[1])
                        if min(pos[0],pos[1]) >= 0 and max(pos[0] , pos[1]) <= 200 and pos != paths[target_robot][track_index[target_robot]+1] and pos not in obstacles and pos_1 not in robot_next_pos and pos_1 not in now_position: # 测试时修改为live_points_1，同时传入
                            if track_index[target_robot] >= 1 and pos == paths[target_robot][track_index[target_robot]-1]:
                                continue
                            # print(target_robot , track_index[target_robot] , pos.x , pos.y , bots[target_robot].x , bots[target_robot].y)
                            wait_pos.append(pos)
                            left_right_flag = True
                    if not left_right_flag: # 没有左右的执行回退
                        track_index[target_robot] = max(0 , track_index[target_robot]-1)
                        reback_status[target_robot] = 3
                        # if track_index[target_robot] > 0:
                        #     tmp_step = min(reback_step , track_index[target_robot])
                        #     next_step = []
                        #     for i in range(track_index[target_robot]-1 , track_index[target_robot]-1-tmp_step , -1):
                        #         next_step.append(paths[target_robot][i])
                        #     for i in range(track_index[target_robot]-tmp_step+1 , track_index[target_robot]+1):
                        #         next_step.append(paths[target_robot][i])
                        #     # 将list插入到路径中
                        #     paths[target_robot].insert(track_index[target_robot]+1 , next_step)
                    else:
                        pos = wait_pos[0]
                        waits[target_robot] = wait_pos
                        paths[target_robot].insert(track_index[target_robot] , pos)
                        reback_status[target_robot] = 2 # 发生过左右的情况
                collision_flag = True

                    # # 更新robot_next_pos
                if isinstance(paths[index][track_index[index]] , tuple) and len(paths[target_robot][track_index[target_robot]]) > 1:
                    robot_next_pos[target_robot] = Node(paths[target_robot][track_index[target_robot]][0] , paths[target_robot][track_index[target_robot]][0])
    # print(track_index[0],track_index[1],track_index[2],track_index[3],track_index[4],track_index[5],track_index[6],track_index[7],track_index[8],track_index[9],file=sys.stderr)
    # 检测计算得到的下一个前往的点是否交叉
    # for i in range(10):
    #     for j in range(10):
    #         if i == j:
    #             continue
    #         if track_index[i] < len(paths[i]) and track_index[j] < len(paths[j]) and ((paths[i][track_index[i]] == (bots[j].x , bots[j].y) and paths[j][track_index[j]] == (bots[i].x , bots[i].y)) or (paths[i][track_index[i]] == paths[j][track_index[j]])):
    #             print(id, file=sys.stderr)
    #             print(paths[i], file=sys.stderr)
    #             print(paths[j], file=sys.stderr)     
    #             print(i,j,paths[i][track_index[i]],paths[j][track_index[j]], file=sys.stderr)
    #             print(id,file=sys.stderr)
    #             print(reback_status,file=sys.stderr)
    #             print(nums,file=sys.stderr)
    # 计算是否能执行
    # for i in range(10):
    #     if track_index[i] < len(paths[i]) and isinstance(paths[i][track_index[i]] , tuple) and len(paths[i][track_index[i]]) == 2:
    #         length = abs(bots[i].x-paths[i][track_index[i]][0]) + abs(bots[i].y-paths[i][track_index[i]][1])
    #         if length > 1:
    #             print(id,i,bots[i].x, bots[i].y, paths[i][track_index[i]], bots[i].status,file=sys.stderr)
    # print(track_index[0],track_index[1],track_index[2],track_index[3],track_index[4],track_index[5],track_index[6],track_index[7],track_index[8],track_index[9],file=sys.stderr)
    # print(len(paths[0]),len(paths[1]),len(paths[2]),len(paths[3]),len(paths[4]),len(paths[5]),len(paths[6]),len(paths[7]),len(paths[8]),len(paths[9]),file=sys.stderr)
    return track_index,paths