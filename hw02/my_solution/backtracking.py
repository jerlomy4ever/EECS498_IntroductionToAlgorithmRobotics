#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from cvxopt import matrix, solvers


def f(x):
    return np.exp(0.5*x + 1) + np.exp(-0.5*x - 0.5) + 5*x

def plot():
    xvals = np.arange(-10, 10, 0.01) # Grid of 0.01 spacing from -10 to 10
    yvals = f(xvals) # Evaluate function on xvals
    plt.plot(xvals, yvals) # Create line plot with yvals against xvals
    plt.show() #show the plot


def backtracking(fun,gradient,x,alpha=0.1,beta=0.6):
    # fun : f(x)
    direction = -1* gradient
    t=1
    while fun(x+t*direction)>fun(x)+alpha*t*gradient*direction:
        t=beta*t
    return t



def gradient_decent(fun,grad_f,x,alpha=0.1,beta=0.6,epsilon=0.0001):
    x_list = np.array([x])
    x_new=x
    gradient=grad_f(x_new)
    while np.abs(gradient)>epsilon:
        t=backtracking(fun,gradient,x_new)
        x_new=x_new-t*gradient
        x_list =np.append(x_list ,[x_new])
        gradient=grad_f(x_new)
    return x_list


def Newton(fun,grad_f,He,x,alpha=0.1,beta=0.6,epsilon=0.0001):
    x_list = np.array([x])
    x_new=x
    gradient=grad_f(x_new)
    Hession=He(x_new)
    decrement=(gradient*gradient/Hession)
    step=-1*gradient/Hession
    while 1 :
        t=backtracking(fun,-1*step,x_new)
        x_new=x_new+t*step
        x_list =np.append(x_list ,[x_new])
        gradient=grad_f(x_new)
        Hession=He(x_new)
        decrement=(gradient*gradient/Hession)
        step=-1*gradient/Hession
        if decrement/2<=epsilon :
            break
    return x_list

def function(x):
    return 5*x+np.exp(1+0.5*x)+np.exp(-0.5-0.5*x)

def grad_f(x):
    return 5+0.5*np.exp(1+0.5*x)-0.5*np.exp(-0.5-0.5*x)

def He(x):
    return 0.5*0.5*np.exp(1+0.5*x)+0.5*0.5*np.exp(-0.5-0.5*x)

def main():
    plt.figure(0)
    xvals = np.arange(-10, 10, 0.01) # Grid of 0.01 spacing from -10 to 10
    yvals = f(xvals) # Evaluate function on xvals
    plt.plot(xvals, yvals,'k') # Create line plot with yvals against xvals
    GD=gradient_decent(function,grad_f,5)
    yvals = f(GD)
    plt.plot(GD, yvals,'r')
    New=Newton(function,grad_f,He,5)
    yvals = f(New)
    plt.plot(New, yvals,'m')
    plt.figure(1)
    xvals = np.arange(GD.shape[0],step=1)
    yvals = f(GD)
    plt.plot(xvals,yvals,'r')
    xvals = np.arange(New.shape[0],step=1)
    yvals = f(New)
    plt.plot(xvals,yvals,'m')
    plt.show() #show the plot



if __name__=='__main__':
    main()
