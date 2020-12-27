#!/usr/bin/env python
import utils
import numpy
###YOUR IMPORTS HERE###
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
###YOUR IMPORTS HERE###


def main():

    #Import the cloud
    pc = utils.load_pc('cloud_pca.csv')
    ###YOUR CODE HERE###
    # Show the input point cloud
    utils.view_pc([pc])

    #Rotate the points to align with the XY plane
    array=numpy.array(pc) #(200,3,1)
    mu=numpy.mean(array,axis=0) #(3,1)
    array=array-mu
    x=array.reshape(array.shape[0],array.shape[1])
    Y=array/numpy.sqrt(array.shape[0]-1)
    Y=Y.reshape((Y.shape[0],Y.shape[1]))
    u, s, vh = numpy.linalg.svd(Y, full_matrices=False)
    v=numpy.transpose(vh)
    x_new=numpy.matmul(x,v)
    x_new=x_new.reshape(x_new.shape[0],x_new.shape[1],1)
    #Show the resulting point cloud
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d(-1.5,1.5)
    ax.set_ylim3d(-1.5,1.5)
    ax.set_zlim3d(-1,1)
    for i in range(x_new.shape[0]):
        ax.scatter(x_new[i][0][0], x_new[i][1][0], x_new[i][2][0],color='b', marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Rotate the points to align with the XY plane\nV_transpose : '+str(v[0]))


    #Rotate the points to align with the XY plane AND eliminate the noise
    v=numpy.transpose(vh)[:,0:2]
    x_new=numpy.matmul(x,v)
    x_new=x_new.reshape(x_new.shape[0],x_new.shape[1],1)
    #Show the resulting point cloud
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.set_xlim3d(-1.5,1.5)
    ax2.set_ylim3d(-1.5,1.5)
    ax2.set_zlim3d(-1,1)
    for i in range(x_new.shape[0]):
        ax2.scatter(x_new[i][0][0], x_new[i][1][0],color='b', marker='o')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title('Rotate the points to align with the XY plane AND eliminate the noise\nV_transpose : '+str(v[0]))


    # fit a plane to the cloud and draw that plane in green in a plot with the point cloud.
    plane_x=numpy.arange(-2,2,0.2)
    plane_y=numpy.arange(-1,1,0.2)
    xv, yv = numpy.meshgrid( plane_x,  plane_y, sparse=False, indexing='ij')
    new_plane=numpy.zeros((3,xv.shape[0]*xv.shape[1]))
    index=0
    for i in range(xv.shape[0]):
        for j in range(xv.shape[1]):
            index=j+i*xv.shape[1]
            new_plane[0][index]=xv[i,j]*v[0,0]  + yv[i,j]*v[0,1] 
            new_plane[1][index]=xv[i,j]*v[1,0]  + yv[i,j]*v[1,1] 
            new_plane[2][index]=xv[i,j]*v[2,0]  + yv[i,j]*v[2,1] 
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111, projection='3d')
    ax3.set_xlim3d(-2,2)
    ax3.set_ylim3d(-2,2)
    for i in range(x.shape[0]):
        ax3.scatter(x[i][0], x[i][1], x[i][2],color='b', marker='o')
    v=numpy.transpose(vh)
    normal=v[:,-1]
    normal=normal.reshape([3,1])
    pt=numpy.array([0,0,0])
    pt=pt.reshape([3,1])
    utils.draw_plane(fig3, normal, pt,color=(0.0, 1, 0.0, 0.3))
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.set_title('fit a plane to the cloud and draw that plane in green in a plot with the point cloud\nV_transpose : '+str(v[0]))

    plt.show()
    ###YOUR CODE HERE###


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
