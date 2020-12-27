#!/usr/bin/env python
from cvxopt import matrix, solvers

def lp():
    A = matrix([ [1.6, 3.5, 0.1, 2.3,6.1,1.0,0.0,0.0,0.0], [7.2, 2.1, 7.1, 3.2,0.1,0.0,1.0,0.0,0.0],[3.7,3.2,2.9,3.4,4.9,0.0,0.0,1.0,0.0],[0.1,0.15,0.1,0.15,0.1,0.0,0.0,0.0,1.0] ])
    A=-1*A
    b = matrix([ 51.0, 48.0, 202.0, 120.0,229.0,0.0,0.0,0.0,0.0 ])
    b=-1*b
    c = matrix([ 75.0, 128.0,70.0,34.0 ]) #this is the optimization function
    sol=solvers.lp(c,A,b)
    print(sol['x'])


def main():
   lp()


if __name__=='__main__':
    main()
