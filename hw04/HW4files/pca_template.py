#!/usr/bin/env python
import utils
import numpy
###YOUR IMPORTS HERE###

###YOUR IMPORTS HERE###


def main():

    #Import the cloud
    pc = utils.load_pc('cloud_pca.csv')


    ###YOUR CODE HERE###
    # Show the input point cloud
    utils.view_pc([pc])

    #Rotate the points to align with the XY plane



    #Show the resulting point cloud


    #Rotate the points to align with the XY plane AND eliminate the noise


    # Show the resulting point cloud

    ###YOUR CODE HERE###


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
