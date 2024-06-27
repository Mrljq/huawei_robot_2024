# 计算逻辑，传回机器人下一次运动动作



import sys

def robots_movation(robot_num):
    for i in range(robot_num):
        print("move", i, 1)
        sys.stdout.flush()


def robot_up(robot_id):
    print("move", robot_id, 2)

def robot_down(robot_id):
    print("move", robot_id, 3)

def robot_left(robot_id):
    print("move", robot_id, 1)

def robot_right(robot_id):
    print("move", robot_id, 0)

def robot_get_goods(robot_id):
    print("get", robot_id)


def robot_pull_goods(robot_id):
    print("pull",robot_id)



def boat_ship(boat_id,berth_id):
    print("ship",boat_id,berth_id)

def boat_go_virtual(boat_id):
    print("go",boat_id)




def robot_go_next(robot,next_x,next_y):
    if robot.x==next_x and robot.y+1==next_y:
        robot_right(robot.id)
    
    elif robot.x==next_x and robot.y-1==next_y:
        robot_left(robot.id)

    elif robot.x+1==next_x and robot.y==next_y:
        robot_down(robot.id)

    elif robot.x-1==next_x and robot.y==next_y:
        robot_up(robot.id)