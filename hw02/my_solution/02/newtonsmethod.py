#!/usr/bin/env python
import numpy as np
from backtracking import backtracking


def Newton(fun,grad,He,x,alpha=0.1,beta=0.6,epsilon=0.0001):
    # fun : the objective function
    # grad : the gradient of the objective function
    # He: the Hessian of the objective function
    # x : initial point
    x_list = np.array([x])
    x_new=x
    gradient=grad(x_new)
    Hession=He(x_new)
    decrement=(gradient*gradient/Hession)
    step=-1*gradient/Hession
    while 1 :
        t=backtracking(fun,step,x_new)
        x_new=x_new+t*step
        x_list =np.append(x_list ,[x_new])
        gradient=grad(x_new)
        Hession=He(x_new)
        decrement=(gradient*gradient/Hession)
        step=-1*gradient/Hession
        if decrement/2<=epsilon :
            break
    return x_list

