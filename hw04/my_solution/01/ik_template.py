#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy
#### YOUR IMPORTS GO HERE ####

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
        jointnames = ['torso_lift_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0.24,1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

#set active DOF values from a numpy matrix
def SetActiveDOFValuesNPMatrix(robot,qmat):
    qo = [q.item(i) for i in range(0,qmat.shape[1])]
    robot.SetActiveDOFValues(qo)


#returns the end effector transform in the world frame
def GetEETransform(robot,activedofvalues=None):
    if activedofvalues != None:
        robot.SetActiveDOFValues(activedofvalues);
    manip = robot.GetActiveManipulator()
    return manip.GetEndEffectorTransform()

#returns the joint axis in the world frame
def GetJointAxis(robot,jointname):
    return robot.GetJoint(jointname).GetAxis(0)

#returns the joint position in the world frame
def GetJointPosition(robot,jointname):
    return robot.GetJoint(jointname).GetAnchor()

def GetTranslationJacobian(robot,jointnames):
    J = numpy.zeros((3,robot.GetActiveDOF()))
    ### YOUR CODE HERE ###
    TF=GetEETransform(robot)
    x=numpy.array([TF[0][3],TF[1][3],TF[2][3]])
    start_index=0
    for joint in jointnames:
        anchor=GetJointPosition(robot,joint)
        axis=GetJointAxis(robot,joint)
        J[:,start_index]=numpy.cross(axis,x-anchor)
        start_index=start_index+1 
    ### YOUR CODE HERE ###
    return J

def GetJpinv(J):
    ### YOUR CODE HERE ###
    small_c=0.001
    Jpinv = numpy.matmul(numpy.transpose(J),numpy.linalg.inv(small_c*numpy.eye(3)+numpy.matmul(J,numpy.transpose(J))))
    ### YOUR CODE HERE ###
    return Jpinv


def iter_jaco(robot,target,start,jointnames,u,l,handles,threshold=0.01,alpha=pi):
    ## only one task
    ## no secondary task
    q_curr=start
    x_cur = numpy.zeros(3)

    SetActiveDOFValuesNPMatrix(robot,q_curr)
    tf_last=GetEETransform(robot)

    while True:
        SetActiveDOFValuesNPMatrix(robot,q_curr)
        tf=GetEETransform(robot)
        handles.append(env.drawlinestrip(points=array(((tf[0][3],tf[1][3],tf[2][3]),(tf_last[0][3],tf_last[1][3],tf_last[2][3]))),
                                           linewidth=3.0,
                                           colors=array(((1,0,0),(1,0,0)))))
        tf_last=tf

        x_cur[0]=GetEETransform(robot)[0][3]
        x_cur[1]=GetEETransform(robot)[1][3]
        x_cur[2]=GetEETransform(robot)[2][3]
        x_dot=target-x_cur
        error=numpy.linalg.norm(x_dot)
        if error<threshold:
            return q_curr
        J=GetTranslationJacobian(robot,jointnames)
        Jpinv=GetJpinv(J)
        q_dot=numpy.matmul(Jpinv,x_dot)
        q_dot_norm=numpy.linalg.norm(q_dot)
        if q_dot_norm>alpha:
            q_dot=(alpha/q_dot_norm)*q_dot
        for i in range(q_curr.shape[1]):
            q_curr[0][i]=q_curr[0][i]+q_dot[i]
            if q_curr[0][i]>u[i]:
                q_curr[0][i]=u[i]
            if q_curr[0][i]<l[i]:
                q_curr[0][i]=l[i]
    return []


def iter_jaco_second(robot,target,start,jointnames,u,l,handles,threshold=0.01,alpha=pi,beta=0.08):
    ## w/ secondary task
    q_curr=start
    x_cur = numpy.zeros(3)
    SetActiveDOFValuesNPMatrix(robot,q_curr)
    tf_last=GetEETransform(robot)
    while True:
        SetActiveDOFValuesNPMatrix(robot,q_curr)
        tf=GetEETransform(robot)
        handles.append(env.drawlinestrip(points=array(((tf[0][3],tf[1][3],tf[2][3]),(tf_last[0][3],tf_last[1][3],tf_last[2][3]))),
                                           linewidth=3.0,
                                           colors=array(((1,0,0),(1,0,0)))))
        tf_last=tf
        x_cur[0]=GetEETransform(robot)[0][3]
        x_cur[1]=GetEETransform(robot)[1][3]
        x_cur[2]=GetEETransform(robot)[2][3]
        x_dot=target-x_cur
        error=numpy.linalg.norm(x_dot)
        if error<threshold:
            return q_curr
        J=GetTranslationJacobian(robot,jointnames)
        Jpinv=GetJpinv(J)
        q_dot=numpy.matmul(Jpinv,x_dot)
        prefix=beta*(numpy.eye(q_dot.shape[0])-numpy.matmul(Jpinv,J))
        q2=numpy.zeros(q_dot.shape[0])
        for i in range(q_dot.shape[0]):
            if abs(q_curr[0][i]-u[i])>abs(q_curr[0][i]-l[i]):
                q2[i]=(q_curr[0][i]-l[i])
            else:
                q2[i]=(q_curr[0][i]-u[i])

        prefix=numpy.matmul( prefix,q2)
        q_dot=q_dot+prefix
        q_dot_norm=numpy.linalg.norm(q_dot)
        if q_dot_norm>alpha:
            q_dot=(alpha/q_dot_norm)*q_dot
        for i in range(q_curr.shape[1]):
            q_curr[0][i]=q_curr[0][i]+q_dot[i]
            if q_curr[0][i]>u[i]:
                q_curr[0][i]=u[i]
            if q_curr[0][i]<l[i]:
                q_curr[0][i]=l[i]
    return []    


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from environment XML file
    env.Load('pr2only.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms and raise torso
    tuckarms(env,robot);
  
    #set start config
    robot.SetActiveManipulator('leftarm')
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      


    targets = [[-0.15070158,  0.47726995,  1.56714123],
           [-0.36535318,  0.11249, 1.08326675],
           [-0.56491217,  0.011443, 1.2922572 ],
           [-1.07012697,  0.81909669,  0.47344636],
           [-1.11050811,  0.97000718,  1.31087581]]
    doflimits = robot.GetActiveDOFLimits() #make sure q doesn't go past these limits
    q = numpy.zeros((1,robot.GetActiveDOF())) #start at this configuration
    start = time.clock()
    with env:
        handles = [] #graphics handles for plotting
        SetActiveDOFValuesNPMatrix(robot,q)

        ### YOUR CODE HERE ###
        target = targets[4] ###pick your target here
        #draw the target point in blue
        handles.append(env.plot3(points=array(target), pointsize=15.0, colors=array((0,0,1)) )) 
        t_np=numpy.array(target)
        l=robot.GetActiveDOFLimits()[0]
        u=robot.GetActiveDOFLimits()[1]

        for i in range(l.shape[0]):
            if l[i]<-pi:
                l[i]=-pi

            if u[i]>pi:
                u[i]=pi

        output=iter_jaco(robot,t_np,q,jointnames,u,l,handles)
        if output!=[]:
            print output
            SetActiveDOFValuesNPMatrix(robot,output)
        ### YOUR CODE HERE ###

    end = time.clock()
    print "Time : "
    print end-start
    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    raw_input("Press enter to exit...")
    env.Destroy()
