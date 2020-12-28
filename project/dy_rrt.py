#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from scipy.spatial.transform import Rotation as R ##need installasion
from heapq import*
from scipy import spatial
import random
import math
from scipy.integrate import quad
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['r_ur5_arm_shoulder_lift_joint','l_ur5_arm_shoulder_lift_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([-2.35,-0.8]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]
    if path==[]:
	    return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')	
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
	    traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

def get_primitives():
    n=3 
    m=3
    primitives=numpy.zeros((2,n*m))

    for i in range(n):
        for j in range(m):
            primitives[0][i*m+j]=i-1 # acceleration
                                        # -1 : hit the brake 
                                        # 0 : maintain the speed
                                        # 1 : accelerate
            primitives[1][i*m+j]=j-1 # front wheel steering angle
                                        # -1 : right turn
                                        # 0 : straight
                                        # 1 : left turn
    return primitives

def Halton (index,b) :
    # Halton sequence
    # get a point between 0 and 1
    f=1.0
    r=0.0
    while index>0:
        f=f/b
        r=r+f*(index%b)
        index=math.floor(index/b)
    return r

def fRand_Halton(fMin,fMax,index):
    # get a configuration point from Halton sequence
    cur_goal=numpy.zeros(6)
    p=[2,3,5,7,11,13]
    for i in range(6):
        cur_goal[i]=fMin[i] + Halton (index,p[i]) * (fMax[i] - fMin[i])
    return cur_goal


def d(s1,s2):
    # euclidian distance 
    error=s2-s1
    while error[0][2]>pi:
        error[0][2]=error[0][2]-2*pi
    while error[0][2]<-1*pi:
        error[0][2]=error[0][2]+2*pi
    distance=numpy.linalg.norm(error)

    return distance

def x_dot(t,v0,u1,beta,theta0,lr=0.54):
    # kinematic bicycle model
    # t : time from the beginning of the time interval
    # v0 : the speed of the center of mass at the beginning of the time interval
    # u1 : acceleration 
    # beta : slip angle at the cemter of mass
    # theta0 : the orientation at the beginning of the time interval
    # lr : the length from the center of mass to the center of rear wheel
    return (v0+u1*t)*numpy.cos(theta0+(numpy.sin(beta)/lr)*(v0*t+0.5*u1*t*t)+beta)

def y_dot(t,v0,u1,beta,theta0,lr=0.54):
    # kinematic bicycle model
    # t : time the beginning of the time interval
    # v0 : the speed of the center of mass at the beginning of the time interval
    # u1 : acceleration 
    # beta : slip angle at the cemter of mass
    # theta0 : the orientation at the beginning of the time interval
    # lr : the length from the center of mass to the center of rear wheel
    return (v0+u1*t)*numpy.sin(theta0+(numpy.sin(beta)/lr)*(v0*t+0.5*u1*t*t)+beta)

def forward_model(u1,u2,state,time_step,lf=0.54,lr=0.54):
    # kinematic bicycle model
    # u1 : acceleration 
    # u2 : front wheel steering angle
    # state : [x, y, theta, x_dot, y_dot, theta_dot]
    # time_step : how long a time interval is
    # lf : the length from the center of mass to the center of front wheel
    # lr : the length from the center of mass to the center of rear wheel
    new_state=numpy.zeros((1,6))
    beta=numpy.arctan( numpy.tan(u2)*lr/(lr+lf) )
    v0=state[0][3]*state[0][3]+state[0][4]*state[0][4]
    v0=numpy.sqrt(v0)
    v=v0+u1*time_step
    new_state[0][5]=(v/lr)*numpy.sin(beta)
    new_state[0][2]=state[0][2]+(numpy.sin(beta)/lr)*(v0*time_step+0.5*u1*time_step*time_step)
    while new_state[0][2]>pi :
        new_state[0][2]=new_state[0][2]-2*pi
    while new_state[0][2]<-1*pi :
        new_state[0][2]=new_state[0][2]+2*pi

    new_state[0][3]=v*numpy.cos(new_state[0][2]+beta)
    new_state[0][0]=state[0][0]+quad(x_dot,0,time_step,args=(v0,u1,beta,state[0][2]))[0]
    

    new_state[0][4]=v*numpy.sin(new_state[0][2]+beta)
    new_state[0][1]=state[0][1]+quad(y_dot,0,time_step,args=(v0,u1,beta,state[0][2]))[0]

    return new_state

def get_opt_primitives(env,robot,start,goal,primitives,time_step,xy_step,theta_step,lower,upper,lf=0.54,lr=0.54):
    state=start
    target=goal
    if state.shape[0]>1:
        state=numpy.resize(state,(1,state.shape[0]))
    if target.shape[0]>1:
        target=numpy.resize(target,(1,target.shape[0]))
    error=d(state,target)
    index=0
    u1=primitives[0][index]*xy_step
    u2=primitives[1][index]*theta_step
    new_state=forward_model(u1,u2,state,time_step)
    new_error=d(new_state,target)
    robot.SetTransform([[numpy.cos(new_state[0][2]),-1*numpy.sin(new_state[0][2]),0,new_state[0][0]],[numpy.sin(new_state[0][2]),numpy.cos(new_state[0][2]),0,new_state[0][1]],[0,0,1,0.15],[0,0,0,1]])
    collision=env.CheckCollision(robot)
    if collision :
        new_error=False
    else :
        for i in range(new_state.shape[1]):
            if new_state[0][i]>=upper[i] or new_state[0][i]<=lower[i] :
                new_error=False
                break

    opt_primitives_id=index
    opt_error=new_error
    opt_state=new_state
    index=index+1
    while index<primitives.shape[1]:
        u1=primitives[0][index]*xy_step
        u2=primitives[1][index]*theta_step
        new_state=forward_model(u1,u2,state,time_step)
        new_error=d(new_state,target)
        robot.SetTransform([[numpy.cos(new_state[0][2]),-1*numpy.sin(new_state[0][2]),0,new_state[0][0]],[numpy.sin(new_state[0][2]),numpy.cos(new_state[0][2]),0,new_state[0][1]],[0,0,1,0.15],[0,0,0,1]])
        collision=env.CheckCollision(robot)
        if collision :
            new_error=False
        else :
            for i in range(new_state.shape[1]):
                if new_state[0][i]>=upper[i] or new_state[0][i]<=lower[i] :
                    new_error=False
                    break
        if new_error==False:
            index=index+1
            continue

        if opt_error==False or new_error<opt_error  :
            opt_primitives_id=index
            opt_error=new_error
            opt_state=new_state
        index=index+1
    
    if opt_error==False or error<=opt_error :  
        return False,opt_state,opt_primitives_id,opt_error,error

    return True,opt_state,opt_primitives_id,opt_error,error

def connect(env,robot,start,goal,primitives,time_step,xy_step,theta_step,lower,upper,lf=0.54,lr=0.54):
    ## if path.length==1, there is no extend
    path=start
    init=start
    target=goal
    if goal.shape[0]>1:
        path=numpy.resize(path,(1,path.shape[0]))
        init=numpy.resize(init,(1,init.shape[0]))
        target=numpy.resize(target,(1,target.shape[0]))
    count=0

    while True:
        success,opt_state,_,opt_error,error=get_opt_primitives(env,robot,init,target,primitives,time_step,xy_step,theta_step,lower,upper)

        if  success==False :
            return path

        path=numpy.concatenate((path,opt_state), axis=0)
        init=opt_state
        count=count+1

    return path

def getstr(point):
    ## convert form np.ndarray to string
    if point.shape[0]==1:
        key=str(point[0][0])+","+str(point[0][1])+","+str(point[0][2])+","+str(point[0][3])+","+str(point[0][4])+","+str(point[0][5])
    else :
        key=str(point[0])+","+str(point[1])+","+str(point[2])+","+str(point[3])+","+str(point[4])+","+str(point[5])

    return key

def getpt(key):
    ## convert form string to np.ndarray
    cord=key.split(",")
    point=[float(cord[0]),float(cord[1]),float(cord[2]),float(cord[3]),float(cord[4]),float(cord[5]) ] 
    point=numpy.array(point)
    point=resize(point,(1,6))
    return point

def draw_raw_path(path,robot,handle):
    for i in range(path.shape[0]):
        dof=path[i]
        tf=GetEETransform(robot,dof)
        handles.append(env.plot3(points=array(((tf[0][3],tf[1][3],tf[2][3]))),
                                   pointsize=5.0,
                                   colors=array(((1,0,0)))))

    return

def backtrack(ParentMap,point):
    
    path=resize(point,(1,6))
    key=getstr(path)
    while key in ParentMap:
        key=ParentMap[key]
        point=getpt(key)
        path=numpy.concatenate((point,path), axis=0)
    
    return path


def dy_rrt(env,robot,goalconfig,tolerance,time_step,xy_step,theta_step,renew_scale,handles):
    pose=poseFromMatrix(robot.GetTransform())
    r = R.from_quat(pose[0:4])
    startconfig=[pose[4],pose[5],r.as_euler('zyx', degrees=False)[0],0,0,0]
    start=numpy.array(startconfig)
    handles.append(env.plot3(points=array(((start[0],start[1],0.05))),pointsize=9.0,colors=array(((0,0,0)))))
        
    goal=numpy.array(goalconfig)
    primitives=get_primitives()

    ParentMap={}
    matrix4tree=start
    matrix4tree=resize(matrix4tree,(1,6))
    tree = spatial.KDTree(matrix4tree)
    renew_temp=True

    lower=[-6,-6,-1.0*pi,-1.0,-1.0,-0.5*pi]
    upper=[6,6,1.0*pi,1.0,1.0,0.5*pi]
    nums=1

    while True:
        while True:
            cur_goal=fRand_Halton(lower,upper,nums)
            nums=nums+1
            if numpy.random.uniform()<0.1: ## bias
                cur_goal=goal
                
            nearest_point=tree.data[ tree.query(cur_goal)[1]]
            extend=connect(env,robot,nearest_point,cur_goal,primitives,time_step,xy_step,theta_step,lower,upper)

            if extend.shape[0]>1:
                break
        
        key_p=getstr(nearest_point)
        for i in range(extend.shape[0]-1):
            key_c=getstr(extend[i+1])
            ParentMap[key_c]=key_p
            key_p=key_c

            handles.append(env.drawlinestrip(points=array(((nearest_point[0],nearest_point[1],0.05),(extend[i+1][0],extend[i+1][1],0.05))),
                                           linewidth=3.0,
                                           colors=array(((1,1,1),(1,1,1)))))
            nearest_point=extend[i+1]
        

        error=extend[-1]-goal
        error_count=0
        for i in range(error.shape[0]):
            if error[i]<tolerance[i] and error[i]>-1*tolerance[i]:
                error_count=error_count+1
        if error_count==6:
            return backtrack(ParentMap,extend[-1])
        ## use renew_temp to decide append or assign new matrix
        ## True:temp_matrix=extend[1:] 
        ## False:temp_matrix=numpy.concatenate((temp_matrix,extend[1:]), axis=0)
        if renew_temp==True:
           temp_matrix=extend[1:]
           renew_temp=False
        else:
            temp_matrix=numpy.concatenate((temp_matrix,extend[1:]), axis=0) 
        if temp_matrix.shape[0]>matrix4tree.shape[0]*0.01*renew_scale:
            renew_temp=True
            matrix4tree=numpy.concatenate((matrix4tree,temp_matrix), axis=0)
            tree = spatial.KDTree(matrix4tree)

    return []

def draw_raw_path(path,robot,handle):
    for i in range(path.shape[0]):
        handles.append(env.plot3(points=array(((path[i][0],path[i][1],0.05))),
                                   pointsize=5.0,
                                   colors=array(((1,0,0)))))

    return


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('Sturn.env.xml')
    #env.Load('easy.xml')
    time.sleep(0.1)
    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the robot's arms for driving
    tuckarms(env,robot);

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        goalconfig = [ 5, 5 ,0,0,0,0] # x y theta x_dot y_dot theta_dot 
        tolerance = [0.5,0.5,pi/8,10,10,10]
        handles=[]
        handles.append(env.drawlinestrip(points=array(((goalconfig[0],goalconfig[1],0.05),(goalconfig[0]+0.7*numpy.cos(goalconfig[2]),goalconfig[1]+0.7*numpy.sin(goalconfig[2]),0.05))),
                                           linewidth=3.0,
                                           colors=array(((0,1,0),(0,1,0)))))
        handles.append(env.drawlinestrip(points=array(((goalconfig[0]+0.4*numpy.cos(goalconfig[2]+0.3),goalconfig[1]+0.4*numpy.sin(goalconfig[2]+0.3),0.05),(goalconfig[0]+0.7*numpy.cos(goalconfig[2]),goalconfig[1]+0.7*numpy.sin(goalconfig[2]),0.05),(goalconfig[0]+0.4*numpy.cos(goalconfig[2]-0.3),goalconfig[1]+0.4*numpy.sin(goalconfig[2]-0.3),0.05))),
                                           linewidth=3.0,
                                           colors=array(((0,1,0),(0,1,0),(0,1,0)))))
        handles.append(env.drawlinestrip(points=array(((goalconfig[0]+numpy.cos(goalconfig[2]+0.78),goalconfig[1]+numpy.sin(goalconfig[2]+0.78),0.05),(goalconfig[0]+numpy.cos(goalconfig[2]-0.78),goalconfig[1]+numpy.sin(goalconfig[2]-0.78),0.05),(goalconfig[0]+numpy.cos(goalconfig[2]-2.35),goalconfig[1]+numpy.sin(goalconfig[2]-2.35),0.05),(goalconfig[0]+numpy.cos(goalconfig[2]+2.35),goalconfig[1]+numpy.sin(goalconfig[2]+2.35),0.05),(goalconfig[0]+numpy.cos(goalconfig[2]+0.78),goalconfig[1]+numpy.sin(goalconfig[2]+0.78),0.05))),
                                           linewidth=3.0,
                                           colors=array(((0,1,0),(0,1,0),(0,1,0),(0,1,0),(0,1,0)))))
        handles.append(env.plot3(points=array(((goalconfig[0],goalconfig[1],0.05))),pointsize=9.0,colors=array(((0,1,0)))))
        start = time.clock()
        xy_step=0.3
        theta_step=pi/8
        time_step=0.1
        output=[]
        renew_scale=15
        output=dy_rrt(env,robot,goalconfig,tolerance,time_step,xy_step,theta_step,renew_scale,handles)
        
        path=[]
        if output!=[]:
            path=output[:,:3]
            draw_raw_path(path,robot,handles)
        else:
            print "Failure"
        ### end ###
        end = time.clock()
        print "Time: ", end - start
        traj = ConvertPathToTrajectory(robot, path)
    if traj != None:
        robot.GetController().SetPath(traj)
            

    waitrobot(robot)
    raw_input("Press enter to exit...")

