#!/usr/bin/env python
import numpy as np
from backtracking import backtracking

def gradient_decent(fun,grad,x,alpha=0.1,beta=0.6,epsilon=0.0001):
    # fun : the objective function
    # grad : the gradient of the objective function
    # x : initial point
    x_list = np.array([x])
    x_new=x
    gradient=grad(x_new)
    while np.abs(gradient)>epsilon:
        t=backtracking(fun,-1*gradient,x_new)
        x_new=x_new-t*gradient
        x_list =np.append(x_list ,[x_new])
        gradient=grad(x_new)
    return x_list
