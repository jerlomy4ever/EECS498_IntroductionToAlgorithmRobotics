#!/usr/bin/env python
import numpy
def main():
    print '5.a \n'
    A = numpy.matrix ( [ [ 0 , 0 , -1 ] , [ 4 , 1 , 1 ] , [ -2, 2 ,1] ] )
    b = numpy.matrix ( [ [ 3 ] , [ 1 ] , [1] ] )
    solve(A,b)
    print '*********** \n'
    print '5.b \n'
    A = numpy.matrix ( [ [ 0 , -2 , 6 ] , [ -4 , -2 , -2 ] , [ 2, 1 ,1] ] )
    b = numpy.matrix ( [ [ 1 ] , [ -2 ] , [0] ] )
    solve(A,b)
    print '*********** \n'
    print '5.c \n'
    A = numpy.matrix ( [ [ 2 , -2 ] , [ -4,3] ] )
    b = numpy.matrix ( [ [ 3 ] , [ -2 ]  ] )
    solve(A,b)

def solve(A,b):
    print 'A = \n',A
    print 'b = \n',b
    r=numpy.linalg.matrix_rank(A)
    print 'rank(A) = ',r
    if(r==A.shape[0]):
        x=numpy.linalg.solve(A,b)
        print'Ax = b has one unique solution ; x = \n',x
        return x
    
    r=numpy.linalg.matrix_rank((A*numpy.transpose(A)))
    if(r==A.shape[0]):
        x=numpy.transpose(A)*numpy.linalg.inv(A*numpy.transpose(A))*b
        print'Ax = b has infinitely many solutions and we can find a solution with minimum ||x|| ; x = \n',x
        return x
    r=numpy.linalg.matrix_rank((numpy.transpose(A)*A))
    if(r==A.shape[0]):
        x=numpy.linalg.inv(numpy.transpose(A)*A)*numpy.transpose(A)*b
        print'Ax = b has no exact solution but has one solution minizing ||Ax-b|| ; x = \n',x
        return x
    
    print 'Ax = b has no solution'


    

if __name__=='__main__':
    main()




