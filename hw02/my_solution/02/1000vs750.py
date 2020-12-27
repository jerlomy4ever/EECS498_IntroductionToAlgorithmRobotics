#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from sgd import *


def main():
    x_start=-5
    t=1
    iteration=1000
    list_1000=np.array([])
    list_750=np.array([])
    for i in range(30):
        list_x=SGD(t,x_start,iteration)
        list_1000 =np.append(list_1000 ,[fsum(list_x[-1])])
        list_x=SGD(t,x_start,750)
        list_750 =np.append(list_750 ,[fsum(list_x[-1])])

    u_1000=np.mean(list_1000)
    u_750=np.mean(list_750)

    v_1000=np.var(list_1000)
    v_750=np.var(list_750)
    print "mean w/ 1000 ietrations :",u_1000 
    print "mean w/ 750 ietrations :",u_750 
    print "variance w/ 1000 ietrations :",v_1000 
    print "variance w/ 750 ietrations :",v_750 


if __name__=='__main__':
    main()

