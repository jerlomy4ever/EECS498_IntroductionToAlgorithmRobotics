#!/usr/bin/env python
def backtracking(fun,direction,x,alpha=0.1,beta=0.6):
    # fun : f(x)
    # direction : descent direction
    gradient = -1 * direction
    t=1
    while fun(x+t*direction)>fun(x)+alpha*t*gradient*direction:
        t=beta*t
    return t
