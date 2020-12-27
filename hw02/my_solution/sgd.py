#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import time
import random
from cvxopt import matrix, solvers
#from backtracking import Newton, gradient_decent

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


def Newton(fun,grad_f,He_f,x,alpha=0.1,beta=0.6,epsilon=0.0001):
    x_list = np.array([x])
    x_new=x
    gradient=grad_f(x_new)
    Hession=He_f(x_new)
    decrement=(gradient*gradient/Hession)
    step=-1*gradient/Hession
    while decrement/2>epsilon :
        t=backtracking(fun,-1*step,x_new)
        x_new=x_new+t*step
        x_list =np.append(x_list ,[x_new])
        gradient=grad_f(x_new)
        Hession=He_f(x_new)
        decrement=(gradient*gradient/Hession)
        step=-1*gradient/Hession

    return x_list

maxi = 10000 #this is the number of functions

def fi(x,i):
    coef1 = 0.01 + (0.5-0.01)*i/maxi
    coef2 = 1 + (6-1)*i/maxi
    return (np.exp(coef1*x + 0.1) + np.exp(-coef1*x - 0.5) - coef2*x)/(maxi/100)

def fiprime(x,i):
    coef1 = 0.01 + (0.5-0.01)*i/maxi
    coef2 = 1 + (6-1)*i/maxi
    return (coef1*np.exp(coef1*x + 0.1) -coef1*np.exp(-coef1*x - 0.5) - coef2)/(maxi/100)

def fiprimeprime(x,i):
    coef1 = 0.01 + (0.5-0.01)*i/maxi
    #coef2 = 1 + (6-1)*i/maxi
    return (coef1*coef1*np.exp(coef1*x + 0.1) +coef1*coef1*np.exp(-coef1*x - 0.5))/(maxi/100)


def fsum(x):
    sum = 0
    for i in range(0,maxi):
       sum = sum + fi(x,i)
    return sum

def fsumprime(x):
    sum = 0
    for i in range(0,maxi):
       sum = sum + fiprime(x,i)
    return sum

def fsumprimeprime(x):
    sum = 0
    for i in range(0,maxi):
       sum = sum + fiprimeprime(x,i)
    return sum

def SGD(t,x,max_iter):    
    x_list = np.array([x])
    x_new=x
    for i in range(max_iter):
        index = random.randint(1,maxi)
        direction=-fiprime(x_new,index)
        x_new=x_new+t*direction
        x_list =np.append(x_list ,[x_new])
    return x_list

def main():
    x_start=-5
    t=1
    iteration=1000
    plt.figure(0)
    xvals = np.arange(-10, 10, 0.01) # Grid of 0.01 spacing from -10 to 10
    yvals = fsum(xvals) # Evaluate function on xvals
    plt.plot(xvals, yvals) # Create line plot with yvals against xvals
    #this is the timing code you should use
    print "Hello world!"

    list_o=np.array([])
    list_750=np.array([])
    for i in range(30):
        list_x=SGD(t,x_start,iteration)
        list_o =np.append(list_o ,[fsum(list_x[-1])])
        list_x=SGD(t,x_start,750)
        list_750 =np.append(list_750 ,[fsum(list_x[-1])])

    u_o=np.mean(list_o)
    u_750=np.mean(list_750)

    v_o=np.var(list_o)
    v_750=np.var(list_750)
    print "original mean " 
    print u_o
    print "\n"
    print u_750
    print "\n"
    print v_o
    print "\n"
    print v_750
    print "\n"

    start = time.clock()
    #YOUR ALGORITHM HERE#
    list_x=SGD(t,x_start,iteration)
    end = time.clock()
    print "Time SGD: ", end - start
    print fsum(list_x[-1])
    plt.figure(1)
    xvals = np.arange(list_x.shape[0],step=1)
    yvals = fsum(list_x)
    plt.plot(xvals,yvals,'r')

    start = time.clock()
    #YOUR ALGORITHM HERE#
    list_x=gradient_decent(fsum,fsumprime,x_start)
    end = time.clock()
    print "Time GD: ", end - start
    print fsum(list_x[-1])
    print list_x.shape[0]
    xvals = np.arange(list_x.shape[0],step=1)
    yvals = fsum(list_x)
    plt.plot(xvals,yvals,'k')

    start = time.clock()
    #YOUR ALGORITHM HERE#
    list_x=Newton(fsum,fsumprime,fsumprimeprime,x_start)
    end = time.clock()
    print "Time Newton: ", end - start
    print fsum(list_x[-1])
    print list_x.shape[0]
    xvals = np.arange(list_x.shape[0],step=1)
    yvals = fsum(list_x)
    plt.plot(xvals,yvals,'b')

    plt.show() #show the plot




if __name__=='__main__':
    main()
