import numpy as np
from model import *
from util import *



robots_next_position=set()
robots_direction_queue=[]

robots_no_self_now_position=[]
# robots_no_self_next_position=[]
def get_robots_no_self_position(robots):
    global robots_no_self_now_position
    # global robots_no_self_next_position

    for count in range(len(robots)):
        tmp_now_position=set()
        tmp_next_position=set()
        for robot in robots:
            if robot.id == count:
                continue
            count+=1
            tmp_now_position.add((robot.x,robot.y))
        
        robots_no_self_now_position.append(tmp_now_position)
        # robots_no_self_next_position.append(tmp_next_position)





five_directions=[(0,0),(-1,0),(1,0),(0,-1),(0,1)] #停 上下左右

def get_robots_direction_queue(robots,track_index,paths,id):


    for robot in robots :
        if len(paths[robot.id])<2:
            robots_direction_queue.append(five_directions)
            continue
        robot_direction_queue=[]
        ahead_direction=get_direction(robot,track_index,paths,id)
        
        back_direction = (ahead_direction[0] * -1, ahead_direction[1] * -1)
        remaining_directions = [direction for direction in five_directions if direction not in [ahead_direction, (0, 0), back_direction]]
        robot_direction_queue.append(ahead_direction)

        robot_direction_queue.append(five_directions[0])
        for it in remaining_directions:
             robot_direction_queue.append(it)
        # robot_direction_queue.append(remaining_directions)
        robot_direction_queue.append(back_direction)
        robots_direction_queue.append(robot_direction_queue)


def get_direction(robot,track_index,paths,id):
    # print(id,robot.id,len(paths[robot.id]),file=sys.stderr)
    direction=(paths[robot.id][track_index[robot.id]][0]-robot.x,paths[robot.id][track_index[robot.id]][1]-robot.y)
    return direction



def is_safe(pos,obs,robot_id):

    if (pos in robots_no_self_now_position[robot_id] ) or (pos in obs) or (pos in robots_next_position):
        return False

    return True




def dfs(visited,robots,obs,actions,id,path=[]):
    
    
    global robots_next_position
    global robots_direction_queue

    if len(path)==len(robots):
        # print(id,path,file=sys.stderr)
        return path
    
    for i,robot in enumerate(robots):
        if visited[i]==1:
            continue
        visited[i]=1
        for action in actions:
            if action=='前进':
                new_pos=(robot.x+robots_direction_queue[i][0][0],robot.y+robots_direction_queue[i][0][1])

            elif action=='停':
                new_pos=(robot.x+robots_direction_queue[i][1][0],robot.y+robots_direction_queue[i][1][1])

            elif action=='左':
                new_pos=(robot.x+robots_direction_queue[i][2][0],robot.y+robots_direction_queue[i][2][1])

            elif action=='右':
                new_pos=(robot.x+robots_direction_queue[i][3][0],robot.y+robots_direction_queue[i][3][1])

            elif action=='后退':
                new_pos=(robot.x+robots_direction_queue[i][4][0],robot.y+robots_direction_queue[i][4][1])


            if is_safe(new_pos,obs,i):

                # for j in range(len(robots)):
                #     if j ==i:
                #         continue
                #     robots_no_self_next_position[j].add(new_pos)
                robots_next_position.add(new_pos)

                for j in range(len(robots)):
                    if j ==i:
                        continue
                    robots_no_self_now_position[j].remove((robot.x,robot.y))

                result = dfs(visited,robots, obs,actions, id,path + [(i, action)])


                robots_next_position.remove(new_pos)


                for j in range(len(robots)):
                    if j ==i:
                        continue
                    robots_no_self_now_position[j].add((robot.x,robot.y))
                # for j in range(len(robots)):
                #     if j ==i:
                #         continue
                #     robots_no_self_next_position[j].remove(new_pos)

                if result:
                    return result
        visited[i]=0

    return None







def plan_actions(paths,robots,track_index,obs,id):

    global robots_next_position
    global robots_direction_queue
    global robots_no_self_now_position


    actions = ['前进', '停', '左', '右', '后退']


    visited=np.zeros(10)
    #获取所有机器人的位置
    get_robots_no_self_position(robots)


    

    #获取机器人方向队列:get_robots_direction_queue
    get_robots_direction_queue(robots,track_index,paths,id)

    # for robot in robots:
    #     if len(paths[robot.id])>1:
    #         print(id,(robot.x,robot.y),(paths[robot.id][track_index[robot.id]]),(robot.x+robots_direction_queue[robot.id][1][0],robot.y+robots_direction_queue[robot.id][1][1]),file=sys.stderr)


    re=dfs(visited,robots,obs,actions,id)

    with open("debug.txt", "a") as file:  # 打开文件以追加模式写入
        file.write(f"id:{id}\n动作列表：{re}\n\n")

    # print("re",re,file=sys.stderr)

    for i,action in re:
        if action=='前进':
           None 
        elif action=='停':
           track_index[i]-=1 

        elif action=='左':
            paths[i].insert(track_index[i],(robots[i].x+robots_direction_queue[i][2][0],robots[i].y+robots_direction_queue[i][2][1]))
            paths[i].insert(track_index[i]+1,(robots[i].x,robots[i].y))

        elif action=='右':
            paths[i].insert(track_index[i],(robots[i].x+robots_direction_queue[i][3][0],robots[i].y+robots_direction_queue[i][3][1]))
            paths[i].insert(track_index[i]+1,(robots[i].x,robots[i].y))


        elif action=='后退':
            paths[i].insert(track_index[i],(robots[i].x+robots_direction_queue[i][4][0],robots[i].y+robots_direction_queue[i][4][1]))

   


    
    robots_next_position=set()
    robots_direction_queue=[]

    robots_no_self_now_position=[]
    
    return track_index,paths




