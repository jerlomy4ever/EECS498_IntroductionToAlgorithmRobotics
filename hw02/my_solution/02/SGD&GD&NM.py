#!/usr/bin/env python
from sgd import *
from gradientdescent import gradient_decent
from newtonsmethod import Newton
import time


def main():
    x_start=-5
    t=1
    iteration=1000
    start = time.clock()
    list_x=SGD(t,x_start,iteration)
    end = time.clock()
    print "SGD time :", end - start
    print "SGD fsum(x) :",fsum(list_x[-1])
    print '*********** '
    start = time.clock()
    list_x=gradient_decent(fsum,fsumprime,x_start)
    end = time.clock()
    print "GD time :", end - start
    print "GD fsum(x) :",fsum(list_x[-1])
    print '*********** '
    start = time.clock()
    list_x=Newton(fsum,fsumprime,fsumprimeprime,x_start)
    end = time.clock()
    print "Newton time :", end - start
    print "Newton fsum(x) :", fsum(list_x[-1])

if __name__=='__main__':
    main()
