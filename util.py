from model import *
import sys
from globaldatas import *
from algorithm import *
ch = []
obstacles=[]
obstacles_set=set()
robots = [Robot() for _ in range(robot_num )]

berths = [Berth() for _ in range(berth_num )]

boats = [Boat(num=_) for _ in range(5)]



goods = allgoods()

#障碍物

# #所有可达点的集合（Node类）
# live_points=[]



# def find_all_live_points():
#     live_points=set_oflivepoints(ch)

def Init():

    for i in range(0, map_size):
        line = sys.stdin.readline().strip()
        ch.append(list(line))
        # line = input()
        # ch.append([c for c in line.split(sep=" ")]) # 读取地图
    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berths[id].x = berth_list[1]
        berths[id].y = berth_list[2]
        berths[id].transport_time = berth_list[3]
        berths[id].loading_speed = berth_list[4]     #初始化泊位
    boat_capacity = int(input())            #初始化轮船容积
    okk = input()   #初始化完成，读取ok


    obstacles=get_obstacles(ch)

    # print(obstacles,file=sys.stderr)
    
    for obstacle in obstacles:
        obstacles_set.add(Node(obstacle[0],obstacle[1]))


    return obstacles ,boat_capacity
    # print("OK")     #输出ok，表示获取到初始化信息
    # sys.stdout.flush()   #清空缓冲区，确保判题器读取到ok
    print(obstacles,file=sys.stderr)



def Input(direction_tables,berths):
    id, money = map(int, input().split(" "))    #帧序号和金钱数

    goods.count_life(id) #更新存活时间
    
    num = int(input())               #新增货物数量K
    for i in range(num):
        x, y, val = map(int, input().split())   #新增K个数量的坐标、价值

        if is_live_goods((x,y),direction_tables,berths):
            goods.add_goods(x, y, val,id)  #新增K个货物


    for i in range(robot_num):    #机器人信息
        robots[i].goods, robots[i].x, robots[i].y, robots[i].status = map(int, input().split())

        robots[i].id=i
        

    for i in range(5):         #船信息
        boats[i].status, boats[i].pos = map(int, input().split())
    okk = input()         #读取判题器发送的OK
    
    return id


def get_live_robots(bots,directions,bers):
    for robot in bots:
        robot.live=False
        for berth in range(len(bers)):
            if directions[berth][(robot.x,robot.y)]!=-1:
                robot.live=True

    return robots


def is_live_goods(pos,directions,bers):
    flag=False
    for berth in range(len(bers)):
        if directions[berth][pos]!=-1:
            flag=True

    return flag     