#!/usr/bin/env python
import utils
import numpy
import time
import random
import matplotlib
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

def add_some_outliers(pc,num_outliers):
    pc = utils.add_outliers_centroid(pc, num_outliers, 0.75, 'uniform')
    random.shuffle(pc)
    return pc

def main():
    #Import the cloud
    time.sleep(0.1)
    pc = utils.load_pc('cloud_pca.csv')

    num_tests = 10
    fig = None
    error_pca=numpy.zeros((1,2))
    error_ransac=numpy.zeros((1,2))
    time_pca=numpy.zeros((1,2))
    time_ransac=numpy.zeros((1,2))

    for i in range(0,num_tests):
        pc = add_some_outliers(pc,10) #adding 10 new outliers for each test
        fig = utils.view_pc([pc])

        ###YOUR CODE HERE###

        ##RANSAC
        start = time.clock()
        array=numpy.array(pc) #(400,3,1)
        array=array.reshape((array.shape[0],array.shape[1]))
        iteration=1000
        threshold=0.2
        N=array.shape[0]/2
        n=array.shape[0]/10
        a=1
        b=1
        c=1
        d=1
        error_best=float('inf') 
        consensus_set=numpy.zeros((1,3))
        inlier_ids=numpy.random.randint(low=0, high=array.shape[0],size=n)
        for k in range(iteration):
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
        end = time.clock()
        this_one=numpy.array( ((i+1)*10,end-start) )
        this_one=this_one.reshape((1,2))
        time_ransac=numpy.concatenate( (time_ransac,this_one) ,axis=0)
        this_one=numpy.array( ((i+1)*10,error_best) )
        this_one=this_one.reshape((1,2))
        error_ransac=numpy.concatenate( (error_ransac,this_one) ,axis=0)
           

        ##PCA
        start = time.clock()
        array=numpy.array(pc) 
        mu=numpy.mean(array,axis=0) 
        array=array-mu
        x=array.reshape(array.shape[0],array.shape[1])
        Y=array/numpy.sqrt(array.shape[0]-1)
        Y=Y.reshape((Y.shape[0],Y.shape[1]))
        u, s, vh = numpy.linalg.svd(Y, full_matrices=False)
        v=numpy.transpose(vh)[:,0:2]
        x_new=numpy.matmul(x,v)
        x_new=x_new.reshape(x_new.shape[0],x_new.shape[1],1)

        x_axis=v[:,0]
        y_axis=v[:,1]
        normal=numpy.cross(x_axis,y_axis)
        v=numpy.transpose(vh)
        normal=v[:,2]
        normal=normal.reshape((3,))

        consensus_set_new=numpy.zeros((1,array[0].shape[0]))
        ids_new=numpy.zeros((1))
        error_best_pca=0
        for j in range(array.shape[0]):

            error_inlier=error(array[j],normal[0], normal[1], normal[2],0)
            if error_inlier<threshold:
                consensus_set_new=numpy.concatenate( (consensus_set_new,numpy.reshape(array[j],(1,array[j].shape[0])) ) ,axis=0)
                ids_new=numpy.append( ids_new,j  )
                error_best_pca=error_inlier+error_inlier*error_inlier



        inlier_pca=consensus_set_new[1:,:]
        inlier_ids_pca=ids_new[1:]
        error_best_pca=numpy.sqrt(error_best_pca)

        end = time.clock()
        this_one=numpy.array( ((i+1)*10,end-start) )
        this_one=this_one.reshape((1,2))
        time_pca=numpy.concatenate( (time_pca,this_one) ,axis=0)
          
        this_one=numpy.array( ((i+1)*10,error_best_pca) )
        this_one=this_one.reshape((1,2))
        error_pca=numpy.concatenate( (error_pca,this_one) ,axis=0)
        matplotlib.pyplot.close(fig)
        ###YOUR CODE HERE###

    ## plot pca
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.set_xlim3d(-1.5,1.5)
    ax2.set_ylim3d(-1.5,1.5)
    ax2.set_zlim3d(-1,1)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title("PCA")

    array=numpy.array(pc) 
    array=array.reshape((array.shape[0],array.shape[1]))
    mu=numpy.mean(array,axis=0) 
    v=numpy.transpose(vh)
    normal=v[:,-1]
    normal=normal.reshape([3,1])
    pt=mu
    pt=pt.reshape([3,1])
    for l in range(array.shape[0]):
        if l in inlier_ids_pca:
            ax2.scatter(array[l][0], array[l][1], array[l][2],color='r', marker='o')

        else:
            ax2.scatter(array[l][0], array[l][1], array[l][2],color='b', marker='o')
    plane_x=numpy.arange(-1.5,1.5,0.2)
    plane_y=numpy.arange(-1.0,1.0,0.2)
    xv, yv = numpy.meshgrid( plane_x,  plane_y, sparse=False, indexing='ij')
    xp=xv*v[0,0]+yv*v[0,1]+mu[0]
    yp=xv*v[1,0]+yv*v[1,1]+mu[1]
    zp=xv*v[2,0]+yv*v[2,1]+mu[2]
    ax2.plot_surface(xp, yp,zp, color=(0.0, 1, 0.0, 0.3))


    # plot ransac
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111, projection='3d')
    ax3.set_xlim3d(-1.5,1.5)
    ax3.set_ylim3d(-1.5,1.5)
    ax3.set_zlim3d(-1,1)
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.set_title("RANSAC")
    for i in range(array.shape[0]):
        if i in inlier_ids:
            ax3.scatter(array[i][0], array[i][1], array[i][2],color='r', marker='o')

        else:
            ax3.scatter(array[i][0], array[i][1], array[i][2],color='b', marker='o')
    plane_x=numpy.arange(-0.6,1.2,0.2)
    plane_y=numpy.arange(-0.4,1.4,0.2)
    xv, yv = numpy.meshgrid( plane_x,  plane_y, sparse=False, indexing='ij')
    zv=(d-a*xv-b*yv)/c
    ax3.plot_surface(xv, yv,zv, color=(0.0, 1, 0.0, 0.3))

    fig4 = plt.figure()
    plt.plot(error_pca[1:,0],error_pca[1:,1],'bo-',label='PCA')
    plt.plot(error_ransac[1:,0],error_ransac[1:,1],'r+-',label='RANSAC')
    plt.title("Error V.S. Number of Outliers")
    plt.legend(['PCA','RANSAC'])
    plt.xlabel("Number of Outliers")
    plt.ylabel("Error")

    fig5 = plt.figure()
    plt.plot(time_pca[1:,0],time_pca[1:,1],'bo-',label='PCA')
    plt.plot(time_ransac[1:,0],time_ransac[1:,1],'r+-',label='RANSAC')
    plt.title("Running Time V.S. Number of Outliers")
    plt.legend(['PCA','RANSAC'])
    plt.xlabel("Number of Outliers")
    plt.ylabel("Running Time (s)")


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
