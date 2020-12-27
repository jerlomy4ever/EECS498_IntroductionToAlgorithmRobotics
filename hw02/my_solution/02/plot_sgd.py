#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from sgd import *

def main():
    x_start=-5
    t=1
    iteration=1000
    plt.figure(0)
    list_x=SGD(t,x_start,iteration)
    xvals = np.arange(list_x.shape[0],step=1)
    yvals = fsum(list_x)
    plt.plot(xvals,yvals,'r')
    plt.title("Objective Funtion vs iterations")
    plt.xlabel("iterations")
    plt.ylabel("fsum(x)")
    plt.show() #show the plot


if __name__=='__main__':
    main()