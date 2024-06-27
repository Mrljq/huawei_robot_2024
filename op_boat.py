import numpy as np
from globaldatas import *
import sys
from util import *
#（1）目的berth id （2）状态 ：0闲置，1确定好船厂还没执行，2为在去船厂路上，3正在取货，4已经可以离开还没执行,5代表在去虚拟点路上。
#（3)轮次（4）转折次数
target_table_boat_berth = np.zeros((5,4),dtype=int)
target_table_boat_berth[:,0] = -1

#代码执行流程为先更新船厂和船状态，然后更新表，再选，再执行

def update_BoatStatus(boats,berths,capacity,now_frame):
    global target_table_boat_berth
    for index_boat in range(5):
        if target_table_boat_berth[index_boat][1] == 5 and boats[index_boat].update_status():
            target_table_boat_berth[index_boat][1] = 0
            target_table_boat_berth[index_boat][3] = 0
            boats[index_boat].idle()
        if (now_frame-1) % 3000 == 0:
            if target_table_boat_berth[index_boat][1] == 0:
                berth_index = select(berths,capacity)
                target_table_boat_berth[index_boat][0] = berth_index
                target_table_boat_berth[index_boat][1] = 1
                berths[berth_index].choosen = 1
                target_table_boat_berth[index_boat][2] += 1 
        #确定是否到船厂
        if target_table_boat_berth[index_boat][1] == 2 and boats[index_boat].update_status():
            target_table_boat_berth[index_boat][1] = 3
        #装货状态
        if target_table_boat_berth[index_boat][1] == 3:
            index_berth = target_table_boat_berth[index_boat][0]
            #判断是否可以继续装货
            if len(berths[index_berth].GoodsOfBerth) > 0 and boats[index_boat].inventory < capacity:
                boats[index_boat].load(berths[index_berth].unload())
            else:
                #先解锁
                berths[index_berth].choosen = 0
                #如果是装满了就直接回去（实际不会出现）
                if boats[index_boat].inventory >= capacity:
                    target_table_boat_berth[index_boat][1] = 4
                    print('id:{}full'.format(now_frame))
                else:#没装满考虑怎么走
                #如果没装满并且没有转移过，找下家
                    if target_table_boat_berth[index_boat][3] == 0:
                        berth_index = select_two(berths,capacity,boats[index_boat].inventory)
                        target_table_boat_berth[index_boat][0] = berth_index
                        target_table_boat_berth[index_boat][1] = 1
                        berths[berth_index].choosen = 1
                        target_table_boat_berth[index_boat][3] = 1
                    #如果转移过只有呆满才能走
                    else: 
                        if (3000 - (now_frame % 3000)) <= berths[index_berth].transport_time + 3:
                            target_table_boat_berth[index_boat][1] = 4  
                            print('boat_id:{},id;{}'.format(index_boat,now_frame),file=sys.stderr)    
                           
    return berths,boats


def select_id1(berths,cap, id):
    global target_table_boat_berth
    metric = np.zeros(10)
    for i in range(10):
        metric[i] = berths[i].transport_time
    for i in range(5):
        i_berth = np.argmax(metric)
        target_table_boat_berth[i][0] = i_berth
        metric[i_berth] = 99999


def select(berths,cap):
    global target_table_boat_berth
    metric = np.zeros(10)
    for i in range(10):
        if berths[i].choosen == 1:
            metric[i] = -1
        else:
            len_goods = min(len(berths[i].GoodsOfBerth),cap)
            value_goods = sum(berths[i].GoodsOfBerth[:len_goods])
            load_time = len_goods / berths[i].loading_speed
            metric[i] = value_goods / (load_time + berths[i].transport_time)
    choosen_berth = np.argmax(metric)
    return choosen_berth

def select_two(berths,cap,boat_v):
    global target_table_boat_berth
    metric = np.zeros(10)
    for i in range(10):
        if berths[i].choosen == 1:
            metric[i] = -1
        else:
            len_goods = min(len(berths[i].GoodsOfBerth),cap-boat_v)
            value_goods = sum(berths[i].GoodsOfBerth[:len_goods])
            load_time = len_goods / berths[i].loading_speed
            metric[i] = value_goods / (500 + load_time + berths[i].transport_time)
    choosen_berth = np.argmax(metric)
    return choosen_berth


def ChooseBerthPair(berths):
    global target_table_boat_berth
    berths_table = np.zeros((10,10),dtype=int)
    berths_table[:,:] = 80000
    for i in range(10):
        x = berths[i].x
        y = berths[i].y
        for i1 in range(i+1,10):
            x_ = berths[i1].x
            y_ = berths[i1].y
            #使用曼哈顿距离
            berths_table[i][i1] = abs(x-x_) + abs(y-y_)
    print(berths_table,file=sys.stderr)
    for i in range(5):
        min_index = np.unravel_index(berths_table.argmin(), berths_table.shape)
        target_table_boat_berth[i][0] = min_index[0]
        target_table_boat_berth[i][1] = min_index[1]
        berths_table[min_index[0],:] = 80000
        berths_table[min_index[1],:] = 80000
        berths_table[:,min_index[0]] = 80000
        berths_table[:,min_index[1]] = 80000