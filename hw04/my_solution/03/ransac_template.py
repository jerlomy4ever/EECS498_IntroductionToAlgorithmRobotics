#!/usr/bin/env python
import utils
import numpy
###YOUR IMPORTS HERE###

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
###YOUR IMPORTS HERE###


def error(p,a_new, b_new, c_new,d_new ):
    nu=abs(a_new*p[0]+b_new*p[1]+c_new*p[2]-d_new)
    de=numpy.sqrt(a_new*a_new+b_new*b_new+c_new*c_new)
    return nu/de

def fit(random_set):
    # ax + by +cz = 1
    d=numpy.ones((random_set.shape[0],))
    x=numpy.matmul(numpy.linalg.pinv(random_set),d) 
    a=x[0]
    b=x[1]
    c=x[2]
    diff=numpy.matmul(random_set,x) -d
    error=numpy.linalg.norm(diff)

    return a,b,c,error


def main():
    #Import the cloud
    pc = utils.load_pc('cloud_ransac.csv')
    ###YOUR CODE HERE###
    # Show the input point cloud
    utils.view_pc([pc])

    #Fit a plane to the data using ransac
    array=numpy.array(pc) #(400,3,1)
    array=array.reshape((array.shape[0],array.shape[1]))
    iteration=1000
    threshold=0.2
    N=array.shape[0]/2
    n=3
    # ax + by +cz = d // d=1
    a=1
    b=1
    c=1
    d=1
    error_best=float('inf') 
    consensus_set=numpy.zeros((1,3))
    inlier_ids=numpy.random.randint(low=0, high=array.shape[0],size=n)

    for i in range(iteration):
        random_ids=numpy.random.randint(low=0, high=array.shape[0],size=n)
        random_set=array[random_ids]
        a_new, b_new, c_new ,error_new=fit(random_set)
        consensus_set_new=numpy.zeros((1,array[0].shape[0]))
        ids_new=numpy.zeros((1))

        for j in range(array.shape[0]):
            if j in  random_ids:
                continue

            if error(array[j],a_new, b_new, c_new,1)<threshold:
                consensus_set_new=numpy.concatenate( (consensus_set_new,numpy.reshape(array[j],(1,array[j].shape[0])) ) ,axis=0)
                ids_new=numpy.append( ids_new,j  )
   
        if consensus_set_new.shape[0]-1>N:
            total=numpy.concatenate( (consensus_set_new[1:,:],random_set ) ,axis=0)
            a_new, b_new, c_new,error_new =fit(total)
            if error_new<error_best:
                error_best=error_new
                a=a_new
                b=b_new
                c=c_new
                consensus_set=total
                inlier_ids=numpy.concatenate( (ids_new[1:],random_ids ) )

    print " a : ",a
    print " b : ",b
    print " c : ",c
    print " d : ",d
    print " error : ",error_best
    print " num of inliers : ", inlier_ids.shape[0]
    #Show the resulting point cloud and the fitted plane
    plane_x=numpy.arange(-0.6,1.2,0.2)
    plane_y=numpy.arange(-0.4,1.4,0.2)
    xv, yv = numpy.meshgrid( plane_x,  plane_y, sparse=False, indexing='ij')
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111, projection='3d')
    ax3.set_xlim3d(-2,2)
    ax3.set_ylim3d(-2,2)
    ax3.set_zlim3d(-2,2)
    for i in range(array.shape[0]):
        if i in inlier_ids:
            ax3.scatter(array[i][0], array[i][1], array[i][2],color='r', marker='o')
        else:
            ax3.scatter(array[i][0], array[i][1], array[i][2],color='b', marker='o')
    zv=(d-a*xv-b*yv)/c
    ax3.plot_surface(xv, yv,zv, color=(0.0, 1, 0.0, 0.3))
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')

    plt.show()
    ###YOUR CODE HERE###
    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
