from util import *
from globaldatas import *
from control import *
from algorithm import *
from track import *
import numpy as np
from target_selector import *
from collision_check import *
from op_boat import *
import time




if __name__ == "__main__":


    obstacles,boat_capacity= Init() #初始化

    #track_index，机器人i下一步即将走的路径索引
    track_index = np.zeros(10, dtype=int)

    #路径列表
    paths=[[] for _ in range(10)]

    #将船厂对设置一下
  

    #方向表，路径长度表
    direction_tables=[]
    paths_lengths=[]


    for i in range(berth_num):
        direction_table,paths_length=find_paths((berths[i].x,berths[i].y), obstacles, map_size, map_size)

        direction_tables.append(direction_table)
        paths_lengths.append(paths_length)

    # ChooseBerthPair(berths)
    # #初始化完成
    print("OK")     #输出ok，表示获取到初始化信息
    sys.stdout.flush()   #清空缓冲区，确保判题器读取到ok
    # with open("debug1.txt", "a") as file:  # 打开文件以追加模式写入
    #     file.write(str(paths_lengths[0].tolist()))


    # print(obstacles,file=sys.stderr)
    # print(paths_lengths[0][15:25,80:90],file=sys.stderr)

    #判断机器人和货物的可达性
    berth_to_robot=np.zeros((len(berths),len(robots)))
    
    all_robots_init=False
    for zhen in range(1,15001):
        start_time = time.time()
       
        # 记录开始时间
        
        id=Input(direction_tables,berths)   #获取每帧开始信息


        ###########################初始化##################################
        # if id %10==0 :
        #     robots=get_live_robots(robots,direction_tables,berths)

        
        
        if not all_robots_init:
            # print("长度为",len(goods.available_goods),file=sys.stderr)
            berth_to_init_goods=np.zeros((len(berths),len(goods.available_goods)))
            robot_to_init_goods=np.zeros((len(robots),len(goods.available_goods)))

            for berth in range(len(berths)):
                for robot in robots:
                    if direction_tables[berth][(robot.x,robot.y)]!=-1:
                        berth_to_robot[berth][robot.id]=1
                for c,good in enumerate(goods.available_goods):
                    if direction_tables[berth][(good[0],good[1])]!=-1:
                        berth_to_init_goods[berth][c]=1
            robot_to_init_goods= np.dot(berth_to_robot.T, berth_to_init_goods)
            # 遍历矩阵，将值为0的位置改为99999
            for i in range(len(robots)):
                for j in range(len(goods.available_goods)):
                    if robot_to_init_goods[i][j] == 0:
                        robot_to_init_goods[i][j] = 99999
        ########################################################################




        #目标选取,获取target_table
        index_good_choosen=robot2goods(goods,robots,berths,paths_lengths,id,robot_to_init_goods,boat_capacity) #需要考虑当货物数目小于10的时候
        # if id <10 :
        #     print(target_table,file=sys.stderr)
        # #更新物体
        goods.reset_available_goods(index_good_choosen)
        # if id ==1 :
            # for berth in berths:
            #     print((berth.x,berth.y),file=sys.stderr)

        # print(target_table,file=sys.stderr)
        
        # print(target_table,file=sys.stderr) 
        #确定路径  机器人死的会报错，没路径
        all_robots_init= True
        for robot in robots:
            if  target_table[robot.id][5]==0:
                all_robots_init= False
            if not robot.live:
                target_table[robot.id][5] = 1
                # print("nihao",robot.id,robot.x,robot.y,file=sys.stderr)
                continue
            start_poisition=(robot.x,robot.y)
            #如果机器人确定了good，则开始寻找good的路径
            if target_table[robot.id][3]==1 :
                goal_poisition=(target_table[robot.id][0],target_table[robot.id][1])

                target_table[robot.id][3]=2


                if  target_table[robot.id][5]==0:

                    # all_robots_init=False

                    # path_bot2berth=direction_tables[target_table[robot.id][2]][start_poisition]
                    # path_good2berth=direction_tables[target_table[robot.id][2]][goal_poisition]
                    # print(robot.id,path_good2berth,file=sys.stderr)

                    paths_Node=astar(Node(start_poisition[0],start_poisition[1]), Node(goal_poisition[0],goal_poisition[1]), obstacles_set)
                    
                    new_path=[]
                    
                    for node in paths_Node:
                        new_path.append((node.x,node.y))

                    paths[robot.id]=new_path

                    target_table[robot.id][5] = 1 
                        
                    track_index[robot.id]=1

                else:
                    # print(robot.id,start_poisition,goal_poisition,"\n",target_table,file=sys.stderr)
                    paths[robot.id]=extract_path(direction_tables[target_table[robot.id][6]], goal_poisition)
                    
                    track_index[robot.id]=1

            #如果机器人到了good拿上good，需要找港口的路径
            elif target_table[robot.id][3]==3 :
                
                target_table[robot.id][3]=4
                # print(start_poisition,target_table,file=sys.stderr)
                paths[robot.id]=extract_path(direction_tables[target_table[robot.id][2]],start_poisition)
                # print(robots[target_table[robot.id][5]],file=sys.stderr)
                paths[robot.id].reverse()
                track_index[robot.id]=1




                # 记录结束时间
        end_time = time.time()

        # 计算运行时间
        elapsed_time = end_time - start_time

        # print(f"{id}: {elapsed_time} 秒",file=sys.stderr)
        # if id < 1000:
        #     with open("debug.txt", "a") as file:  # 打开文件以追加模式写入
        #         file.write(f"id:{id}\n机器人路径:\n{paths[0]},\n机器人位置:{[robots[0].x,robots[0].y]},\n机器人索引:{track_index[0]}\n机器人目标位置{paths[0][track_index[0]]}\n机器人表格:{target_table[0]}\n机器人状态:{(robots[0].goods,robots[0].status)}\n\n\n")  
        #     if abs(robots[0].x-paths[0][track_index[0]][0])+abs(robots[0].y-paths[0][track_index[0]][1])>1:
        #         print(id,file=sys.stderr)




        #碰撞检测
        track_index,paths=detect_collision_1(paths , robots , track_index)

        




        #沿轨迹行驶
        for robot in robots:
            if not robot.live:
                continue
            if target_table[robot.id][3]==2 or target_table[robot.id][3]==4:
                #到了最后一个 不要加1
                track_index[robot.id]=along_path(robot,paths[robot.id],track_index[robot.id])



                
        #是否到达目的地        
        for robot in robots:
            if not robot.live:
                continue
            #如果路径索引是路径的最后一个点，则到达目的地  
            if track_index[robot.id]==len(paths[robot.id]):
                #如果正在去物体的路上，并且到了，则取物体
                if target_table[robot.id][3]==2:
                    robot_get_goods(robot.id)
                    target_table[robot.id][3]=3
                
                #如果正在去港口的路上，并且到了，则放物体
                if target_table[robot.id][3]==4:
                    robot_pull_goods(robot.id)
                    target_table[robot.id][3]=0
                    if robot.goods==1:
                        berths[target_table[robot.id][2]].add_good(target_table[robot.id][4])




#船和船厂
        # if id == 1:
        #     select_id1(berths,boat_capacity,id)
        # #添加函数返回
        # else:
        berths,boats = update_BoatStatus(boats,berths,boat_capacity,id)
        # if id%1000==0:
        #     print(id,target_table_boat_berth,file=sys.stderr)
        
        

        #获得：target_table_boat_berth
       # berths=select_berth(boats,berths,id,boat_capacity)

        # if id%1000==0:
        #     print(len(berths[0].GoodsOfBerth),file=sys.stderr)
            # None
        if id % 1000 == 0:
            n = 0
            n1 =n2= 0
            for i in berths:
                n += sum(i.GoodsOfBerth)
                n2 += sum(i.hisgood)
                n1 += len(i.GoodsOfBerth)
            print('id:{},sum:{},len:{},all:{}'.format(id,n,n1,n2),file=sys.stderr)
            for i in boats:
                print('id:{},boad_left:{}'.format(id,i.inventory),file=sys.stderr)
            print(target_table_boat_berth,file=sys.stderr)
        if id % 15000 == 0:
            for i in range(10):
                print("cap:{},id:{},left_val:{},left_num:{},transtime:{},load:{}".format(boat_capacity,i,sum(berths[i].GoodsOfBerth),len(berths[i].GoodsOfBerth),berths[i].transport_time,berths[i].loading_speed),file=sys.stderr)
            
        # print(boats[1].status,file=sys.stderr)
        # if  boats[1].status==1:
        # print(id,boats[1].status,file=sys.stderr)
            
        #控制船
        for boat in boats:
            if target_table_boat_berth[boat.num][1] == 1:
               # 返回要去的id
                boat_ship(boat.num,target_table_boat_berth[boat.num][0])
                target_table_boat_berth[boat.num][1] =2
            if target_table_boat_berth[boat.num][1] ==4:  
                boat_go_virtual(boat.num)
                target_table_boat_berth[boat.num][1] =5
        
        # 记录结束时间
        end_time = time.time()

        # 计算运行时间
        elapsed_time = end_time - start_time
        if elapsed_time>0.015:
            print(f"{id}:函数运行时间: {elapsed_time} 秒",file=sys.stderr)
        print("OK")
        sys.stdout.flush()
        continue



        




        print("OK")
        sys.stdout.flush()





