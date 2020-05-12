#!/usr/bin/env python
import rospy
import math
import copy

def main():
    tra_path='/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory_goal_func.txt'
    out_put_path='/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory_recompute.txt'
    with open(out_put_path,'w') as f:
        f.write('computed position trajectory'+'\r\n')

    tra=[]
    with open(tra_path,'r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split(' ')))
            tra.append(copy.deepcopy([data[0],data[4],data[5],data[6]]))

    dt=tra[1][0]-tra[0][0]
    current_pos=[0,0,0]
    for i in range(len(tra)):
        new_theta=current_pos[2]+tra[i][3]*dt
        new_x=current_pos[0]+tra[i][1]*dt*math.cos(current_pos[2])-tra[i][2]*dt*math.sin(current_pos[2])
        new_y=current_pos[1]+tra[i][2]*dt*math.cos(current_pos[2])+tra[i][1]*dt*math.sin(current_pos[2])

        with open(out_put_path,'a') as f:
            f.write(str(tra[i][0])+' '+str(current_pos[0])+' '+str(current_pos[1])+' '+str(current_pos[2])+'\r\n')
        
        current_pos=[new_x,new_y,new_theta]


if __name__ == '__main__':
    main()



