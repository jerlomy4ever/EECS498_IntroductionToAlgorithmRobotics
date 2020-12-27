#!/usr/bin/env python
import numpy as np
import random

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

