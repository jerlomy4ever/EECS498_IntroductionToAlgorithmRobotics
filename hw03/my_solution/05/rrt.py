#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy

#### YOUR IMPORTS GO HERE ####
from scipy import spatial
import random
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [joint1_i, joint2_i, joint3_i,...]

    if path==[]:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')	
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeActiveDOFTrajectory(traj,robot)#,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

def tuckarms(env,robot):
    with env:
        jointnames = ['torso_lift_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0.24,1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def GetEETransform(robot,activedofvalues=None):
    #if activedofvalues != None:
    #    robot.SetActiveDOFValues(activedofvalues);
    robot.SetActiveDOFValues(activedofvalues);
    manip = robot.GetActiveManipulator()
    return manip.GetEndEffectorTransform()

def connect(env,robot,start,goal,step=0.05):
    ## if path.length==1, there is no extend
    path=start
    direction=goal-start
    if start.shape[0]>1:
        path= resize(path,(1,6))
        direction= resize(direction,(1,6))
    range_=numpy.linalg.norm(direction)
    direction=direction/range_
    range_=int(ceil(range_/step))
    for i in range(range_):
        if i==range_-1:
            if goal.shape[0]>1:
                path=numpy.concatenate((path,resize(goal,(1,6))), axis=0)
            else:
                path=numpy.concatenate((path,goal), axis=0)
            break
        next_point=start+(i+1)*step*direction
        robot.SetActiveDOFValues(next_point[0])
        if env.CheckCollision(robot):
            break
        path=numpy.concatenate((path,next_point), axis=0)
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
    ## return np.ndarray with size (1,6)
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


def rrt(env,robot,start,goal,step=0.05,renew_scale=15):
    lower,upper = robot.GetActiveDOFLimits()
    lower[4]=-pi
    upper[4]=pi
    upper[3]=pi

    start_np=numpy.array(start)
    goal_np=numpy.array(goal)
    ParentMap={}
    matrix4tree=start_np
    matrix4tree=resize(matrix4tree,(1,6))
    tree = spatial.KDTree(matrix4tree)
    renew_temp=True
    
    while True:
        while True:
            cur_goal=numpy.random.uniform(low=lower, high=upper)
            if numpy.random.uniform()<0.1: ## bias
                cur_goal=goal_np
            nearest_point=tree.data[ tree.query(cur_goal)[1]] 
            extend=connect(env,robot,nearest_point,cur_goal,step)
            if extend.shape[0]>1:
                break
        key_p=getstr(nearest_point)
        for i in range(extend.shape[0]-1):
            key_c=getstr(extend[i+1])
            ParentMap[key_c]=key_p
            key_p=key_c
        if numpy.allclose(extend[-1],goal_np):
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


def smooth(env,path,robot,handles,iterations=150):

    for i in range(iterations):
        idx=numpy.random.uniform(low=[0,0], high=[path.shape[0],path.shape[0]])
        begin=int(numpy.min(idx))
        end=int(numpy.max(idx))
        if begin!=end:
            extend=connect(env,robot,path[begin],path[end])
            if numpy.allclose(extend[-1],path[end]):
                path=numpy.concatenate((path[:begin],extend,path[1+end:]), axis=0)

    for i in range(path.shape[0]):
        dof=path[i]
        tf=GetEETransform(robot,dof)
        handles.append(env.plot3(points=array(((tf[0][3],tf[1][3],tf[2][3]))),
                                   pointsize=5.0,
                                   colors=array(((0,0,1)))))


    return path

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from environment XML file
    env.Load('pr2table.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms and raise torso
    tuckarms(env,robot);
  
    #set start config
    robot.SetActiveManipulator('leftarm')
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      
    startconfig = [0.5, 1.19, -1.548, 1.557, -1.32, -0.1928]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    with env:
        goalconfig = [0.5, 0.33, -1.548, 1.557, -1.32, -0.1928]
        start = time.clock()
        ### YOUR CODE HERE ###
        ### Plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        step_size=0.05
        renew_scale=15
        output=rrt(env,robot,startconfig,goalconfig,step_size,renew_scale)#rrt_connect(env,robot,startconfig,goalconfig,step_size,renew_scale)
        path = [] 
        if output!=[]:
            path=output
            handles = []
            draw_raw_path(path,robot,handles)
            path=smooth(env,path,robot,handles)
        else:
            print "Failure"

        #### END OF YOUR CODE ###
        end = time.clock()
        print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory 
        traj = ConvertPathToTrajectory(robot, path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)

    waitrobot(robot)

    raw_input("Press enter to exit...")
    env.Destroy()
