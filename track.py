from algorithm import *
from util import *
from control import *


'''
path

'''


def along_path(robot,path_of_robot,robot_track_id):
    if robot_track_id<len(path_of_robot):
        robot_go_next(robot,path_of_robot[robot_track_id][0],path_of_robot[robot_track_id][1])
        return robot_track_id+1
    else:
        return robot_track_id

    
