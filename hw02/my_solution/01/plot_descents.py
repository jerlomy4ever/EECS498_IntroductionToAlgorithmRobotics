#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from gradientdescent import gradient_decent
from newtonsmethod import Newton

def f(x):
    return np.exp(0.5*x + 1) + np.exp(-0.5*x - 0.5) + 5*x

def grad(x):
    return 5+0.5*np.exp(1+0.5*x)-0.5*np.exp(-0.5-0.5*x)

def He(x):
    return 0.5*0.5*np.exp(1+0.5*x)+0.5*0.5*np.exp(-0.5-0.5*x)


def plot():
    xvals = np.arange(-10, 10, 0.01) # Grid of 0.01 spacing from -10 to 10
    yvals = f(xvals) # Evaluate function on xvals
    plt.plot(xvals, yvals) # Create line plot with yvals against xvals
    plt.show() #show the plot

def main():
    plt.figure(0)
    xvals = np.arange(-10, 10, 0.01) # Grid of 0.01 spacing from -10 to 10
    yvals = f(xvals) # Evaluate function on xvals
    plt.plot(xvals, yvals,'k') # Create line plot with yvals against xvals
    GD=gradient_decent(f,grad,5)
    yvals = f(GD)
    plt.plot(GD, yvals,'r')
    New=Newton(f,grad,He,5)
    yvals = f(New)
    plt.plot(New, yvals,'m')
    plt.title("Objective Funtion")
    plt.legend(['Objective Funtion','Gradient Descent','Newtons method'])
    plt.xlabel("x")
    plt.ylabel("f(x)")
    plt.figure(1)
    xvals = np.arange(GD.shape[0],step=1)
    yvals = f(GD)
    plt.plot(xvals,yvals,'r')
    xvals = np.arange(New.shape[0],step=1)
    yvals = f(New)
    plt.plot(xvals,yvals,'m')
    plt.title("Objective Funtion vs iterations")
    plt.legend(['Gradient Descent','Newtons method'])
    plt.xlabel("iterations")
    plt.ylabel("f(x)")
    plt.show() #show the plot



if __name__=='__main__':
    main()

