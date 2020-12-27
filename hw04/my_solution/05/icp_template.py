#!/usr/bin/env python
import utils
import numpy
import matplotlib.pyplot as plt
###YOUR IMPORTS HERE###
from scipy import spatial
import random
###YOUR IMPORTS HERE###

def getT(source,tree,c):
    p_bar=0
    q_bar=0
    r=0
    t=0
    for i in range(c.shape[0]):
        if i==0:
            p_bar=source[int(c[i][0])]
            q_bar=tree.data[ int(c[i][1]) ] 
        else:
            p_bar=p_bar+source[int(c[i][0])]
            q_bar=q_bar+tree.data[ int(c[i][1]) ] 

    p_bar=p_bar/c.shape[0]  
    q_bar=q_bar/c.shape[0]
    X=0
    Y=0
    for i in range(c.shape[0]):
        if i==0:
            X=source[int(c[i][0])]-p_bar
            Y=tree.data[ int(c[i][1]) ] -q_bar
            X=X.reshape( (p_bar.shape[0],1) )
            Y=Y.reshape( (q_bar.shape[0],1) )
        else:
            newX=source[int(c[i][0])]-p_bar
            newY=tree.data[ int(c[i][1]) ] -q_bar
            newX=newX.reshape( (p_bar.shape[0],1) )
            newY=newY.reshape( (q_bar.shape[0],1) )

            X=numpy.concatenate( (X,newX) ,axis=1)
            Y=numpy.concatenate( (Y,newY) ,axis=1)
    S=numpy.matmul(X,numpy.transpose(Y))
    u, s, vh = numpy.linalg.svd(S, full_matrices=True)
    left=numpy.transpose(vh)
    right=numpy.transpose(u)
    middle=numpy.eye(S.shape[0])
    middle[-1][-1]=numpy.linalg.det( numpy.matmul(left,right) )
    r=numpy.matmul(left,middle)
    r=numpy.matmul(r,right)
    t=q_bar-numpy.matmul(r,p_bar)

    return r,t


def residual(source,tree,c,r,t):
    error=0
    for i in range(c.shape[0]):
        diff=numpy.matmul(r,source[int(c[i][0])])+t-tree.data[ int(c[i][1]) ] 
        error=error+numpy.linalg.norm(diff)
    return error


def icp(pc_source,pc_target,iterations=100,threshold=0.01):

    source=numpy.array(pc_source)
    source=source.reshape((source.shape[0],source.shape[1]))
    target=numpy.array(pc_target)
    target=target.reshape((target.shape[0],target.shape[1]))
    tree = spatial.KDTree(target)
    pc_output=source
    error=[]
    
    for it in range(iterations):
        c=[]
        for i in range(source.shape[0]):
            this_one=numpy.array( (i,tree.query(source[i])[1]) )
            this_one=this_one.reshape((1,2))
            if c==[]:
                c=this_one
            else:
                c=numpy.concatenate( (c,this_one) ,axis=0)

        r,t=getT(source,tree,c)
        error_new=residual(source,tree,c,r,t)

        if it==0:
            error=numpy.array( (it+1,error_new))
            error=error.reshape((1,2))
        else:
            this_one=numpy.array( (it+1,error_new) )
            this_one=this_one.reshape((1,2))
            error=numpy.concatenate( (error,this_one) ,axis=0)

        if error_new<threshold:
            return pc_output,error

        for i in range(pc_output.shape[0]):
            pc_output[i]=numpy.matmul(r,source[i])+t

        source=pc_output

    return pc_output,error



def main():
    #Import the cloud
    pc_source = utils.load_pc('cloud_icp_source.csv')

    ###YOUR CODE HERE###
    pc_target = utils.load_pc('cloud_icp_target0.csv') # Change this to load in a different target
    pc_output,error=icp(pc_source,pc_target,iterations=100)
    utils.view_pc([pc_source, pc_target], None, ['b', 'r'], ['o', '^'])
    plt.axis([-0.15, 0.15, -0.15, 0.15])

    pc_output=numpy.transpose(pc_output)
    pc_output=pc_output.reshape((pc_output.shape[0],pc_output.shape[1],1))
    output=utils.convert_matrix_to_pc(pc_output)
    utils.view_pc([output, pc_target], None, ['b', 'r'], ['o', '^'])
    plt.axis([-0.15, 0.15, -0.15, 0.15])

    fig4 = plt.figure()
    plt.plot(error[:,0],error[:,1],'r+-')
    plt.title("Error V.S.Iterations")
    plt.xlabel("Iterations")
    plt.ylabel("Error")
    e=error[-1,-1]
    plt.text(4, 6, 'final error : '+str(e), fontsize=15)
    ###YOUR CODE HERE###

    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
