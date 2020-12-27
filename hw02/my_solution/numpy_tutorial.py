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

def lp():
    A = matrix([ [-1.0, -1.0, 0.0, 1.0], [1.0, -1.0, -1.0, -2.0] ])
    b = matrix([ 1.0, -2.0, 0.0, 4.0 ])
    c = matrix([ 2.0, 1.0 ]) #this is the optimization function
    sol=solvers.lp(c,A,b)

    print(sol['x'])


def main():
    lp()
if __name__=='__main__':
    main()
