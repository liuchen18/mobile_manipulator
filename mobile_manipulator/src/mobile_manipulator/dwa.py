#! /usr/bin/env python
import math, copy

class DWA:
    def __init__(self, init_pose=(0, 0, 0),file_path='/home/chen/ws_chen/src/mm_meta_pkg/mobile_manipulator/data/base_trajectory.txt'):
        # Linear velocity limits
        self.MAX_X_VEL = 1.0     # ms^(-1) max speed of each wheel
        self.MAX_X_ACC = 0.5     # ms^(-2) max rate we can change speed of each wheel
        self.MAX_Y_VEL = 1.0     # ms^(-1) max speed of each wheel
        self.MAX_Y_ACC = 0.5     # ms^(-2) max rate we can change speed of each wheel
        # Angular velocity limits
        self.MAX_THETA_VEL = 0.5
        self.MAX_THETA_ACC = 0.5
        # Current linear velocity and angular velocity
        self.current_x_vel = 0.0
        self.current_y_vel = 0.0
        self.current_theta_vel = 0.0
        # Current positions
        self.current_x, self.current_y, self.current_theta = init_pose
        # Parameters for prediction trajectory
        self.dt=0.05
        self.vel_resolution=0.01
        self.current_time=0.0
        self.point_delta_time=0.0
        self.x_position_o=[]
        self.y_position_o=[]
        self.theta_position_o=[]
        self.time_o=[]
        self.last_point_index=0
        self.read_data(file_path)

    def read_data(self,file_path):
        with open(file_path,'r') as f:
            for line in f.readlines()[:]:
                self.x_position_o.append(float(line.split(' ')[1]))
                self.y_position_o.append(float(line.split(' ')[2]))
                self.theta_position_o.append(float(line.split(' ')[3]))
                self.time_o.append(float(line.split(' ')[0]))
        self.point_delta_time=self.time_o[1]-self.time_o[0]

    def state_update(self,time,vel_list,position_list):
        ''' update real time state'''
        self.current_time=time
        self.current_x_vel=vel_list[0]
        self.current_y_vel=vel_list[1]
        self.current_theta_vel=vel_list[2]
        self.current_x=position_list[0]
        self.current_y=position_list[1]
        self.current_theta=position_list[2]
        while (self.time_o[self.last_point_index+1]-time)<0.01:
            self.last_point_index+=1

    def predict_position(self,x_vel_posible,y_vel_posible,theta_possible,delta_time):
        '''this function predict the posible position after delta time using posible velocity,return predicted x,y,theta position'''
        new_theta=self.current_theta+theta_possible*self.dt
        new_x=self.current_x+x_vel_posible*delta_time*math.cos(new_theta)-y_vel_posible*delta_time*math.sin(new_theta)
        new_y=self.current_y+y_vel_posible*delta_time*math.cos(new_theta)+x_vel_posible*delta_time*math.sin(new_theta)

        return new_x,new_y,new_theta

    def sample_velocity(self):
        '''this function sample velocity,return a list that contains all posible velocities'''
        max_x=min(self.MAX_X_VEL,self.current_x_vel+self.MAX_X_ACC*self.dt)
        max_y=min(self.MAX_Y_VEL,self.current_y_vel+self.MAX_Y_ACC*self.dt)
        max_theta=min(self.MAX_THETA_VEL,self.current_theta_vel+self.MAX_THETA_ACC*self.dt)

        min_x=max(-self.MAX_X_VEL,self.current_x_vel-self.MAX_X_ACC*self.dt)
        min_y=max(-self.MAX_Y_VEL,self.current_y_vel-self.MAX_Y_ACC*self.dt)
        min_theta=max(-self.MAX_THETA_VEL,self.current_theta_vel-self.MAX_THETA_ACC*self.dt)

        #print('max x vel'+str(max_x)+' min x vel'+str(min_x))

        sample_vel_result=[]
        for i in range(int((max_x-min_x)/self.vel_resolution)):
            for j in range(int((max_y-min_y)/self.vel_resolution)):
                for k in range(int((max_theta-min_theta)/self.vel_resolution)):
                    sample_vel_result.append(copy.deepcopy([min_x+i*self.vel_resolution,min_y+j*self.vel_resolution,min_theta+k*self.vel_resolution]))
        return sample_vel_result

    def compute_goal_function(self,x_vel_posible,y_vel_posible,theta_vel_possible,point_num):
        '''generate the goal func for every sampled velocity,return a float as goal func'''
        point=[]
        for i in range(point_num):
            point.append(copy.deepcopy([self.x_position_o[self.last_point_index+1+i],self.y_position_o[self.last_point_index+1+i],self.theta_position_o[self.last_point_index+1+i]]))
        point_bias=0.0
        for i in range(point_num):
            delta_time=self.time_o[self.last_point_index+1]-self.current_time+i*self.point_delta_time
            #print('delta time: '+str(delta_time))
            pos_x,pos_y,pos_theta=self.predict_position(x_vel_posible,y_vel_posible,theta_vel_possible,delta_time)
            point_bias+=(abs(point[i][0]-pos_x)+abs(point[i][1]-pos_y)+abs(point[i][2]-pos_theta))*1.0/delta_time
            #if i==0:
            #    print('current predicted error: x: '+str(abs(point[i][0]-pos_x))+' y: '+str(abs(point[i][1]-pos_y))+' theta: '+str(abs(point[i][2]-pos_theta)))

        vel_bias=0#self.dt*0.5*(abs(self.current_x_vel-x_vel_posible)+abs(self.current_y_vel-y_vel_posible)+abs(self.current_theta_vel-theta_vel_possible))
        
        return vel_bias+point_bias

    def get_best_vel(self):
        '''using dwa to get best vel, return x_vel,y_vel,theta_vel'''
        if self.last_point_index>=len(self.x_position_o)-1:
            return 0,0,0
        vel_sample_res=self.sample_velocity()
        print('sample num: '+str(len(vel_sample_res)))
        min_goal_func=float('inf')
        best_x_vel=0
        best_y_vel=0
        best_theta_vel=0
        for i in range(len(vel_sample_res)):
            cur_goal_func=self.compute_goal_function(vel_sample_res[i][0],vel_sample_res[i][1],vel_sample_res[i][2],3)
            if i==0:
                print('rand goal func: '+str(cur_goal_func))
            if cur_goal_func<min_goal_func:
                min_goal_func=cur_goal_func
                best_x_vel=vel_sample_res[i][0]
                best_y_vel=vel_sample_res[i][1]
                best_theta_vel=vel_sample_res[i][2]
        #print(min_goal_func)
        print('min_goao func : '+str(min_goal_func))
        delta_time=self.time_o[self.last_point_index+1]-self.current_time
        pos_x,pos_y,pos_theta=self.predict_position(best_x_vel,best_y_vel,best_theta_vel,delta_time)
        print('current x vel: '+str(self.current_x_vel)+' current y vel: '+str(self.current_y_vel)+' current theta vel: '+str(self.current_theta_vel))
        print('current x :'+str(self.current_x)+ ' predicted x :'+str(pos_x)+' desired x :'+str(self.x_position_o[self.last_point_index+1]))

        return best_x_vel,best_y_vel,best_theta_vel



