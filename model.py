# import numpy as np
# import math
# import sys
# import os
# import threading
# import control
#ght

class Robot:  #机器人
    def __init__(self, robot_id=0,startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX
        self.y = startY
        self.goods = goods #0 表示未携带货物。1 表示携带货物。
        self.status = status #0 表示恢复状态.1 表示正常运行状态
        self.mbx = mbx
        self.mby = mby
        self.live=True
        self.id=robot_id



class Berth:  #船泊
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed
    #设商品队列
        self.GoodsOfBerth = []
        #是否有船选：0 没人选 1 有人选 2有两个人选
        self.choosen = 0
        #提供给选择的物品列表：
        self.GoodsChoosen = []
        self.hisgood=[]

    #机器人装货时请调用这个函数
    def add_good(self,goods_value):
        self.GoodsOfBerth.append(goods_value)
        self.GoodsChoosen.append(goods_value)
        self.hisgood.append(goods_value)
    #实时卸货函数
    def unload(self):
        #如果存量小于装货量，这一帧将剩余的货物量返回并清空列表
        if self.loading_speed > len(self.GoodsOfBerth):
            k = len(self.GoodsOfBerth)
            self.GoodsOfBerth = []
            return k
        else:
            self.GoodsOfBerth = self.GoodsOfBerth[self.loading_speed:]
            return self.loading_speed
    #如果被选中就调用这个函数更新物品列表
    def choose(self,c):
        l = min(len(self.GoodsChoosen),c)
        self.GoodsChoosen = self.GoodsChoosen[l:]
    #如果有船离开调用这个
    def leave(self):
        self.choosen = self.choosen - 1
        if self.choosen == 0:
            self.GoodsChoosen = self.GoodsOfBerth
    

    
   




class Boat:  #船
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        #目标泊位，初始为-1代表虚拟点
        self.pos = pos
        #从input获得的状态2代表厂外等，1是正常（装货或空闲），0是移动中
        self.status = status
        #到达帧
        self.inventory = 0
    #更新船的状态
    def update_status(self):
        return self.status == 1
    #进行装货,这一帧装超就装超无所谓的
    def load(self,num_goods):
        self.inventory = self.inventory + num_goods
    #闲置时进行更新将存货设定为0
    def idle(self):
        self.inventory = 0



class allgoods:
    def __init__(self):
        # self.goods=[]
        self.available_goods = []

    def add_goods(self,x=0,y=0,val=0,id=0):
        # self.goods.append([x,y,val,id])
        self.available_goods.append([x,y,val,id])

    def count_life(self,id):
        if not self.available_goods:
            return  # 如果列表为空，直接返回，无需进入循环
        remaining_goods = []
        for good in self.available_goods:
            if id-good[3]<1000:
                # 添加要保留的商品
                remaining_goods.append(good)

        self.available_goods = remaining_goods
    
    def reset_available_goods(self,ind):
        if len(ind)>0:  # 检查列表是否为空
            self.available_goods = [self.available_goods[i] for i in range(len(self.available_goods)) if i not in ind]
        else:
            # 如果列表为空，则将其设置为一个空列表
            None
    





# Node类
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.parent = None
        self.obstacle = False

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    
    