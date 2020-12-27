#!/usr/bin/env python
import numpy
def main():
    print 'hello world'
    A = numpy.matrix ( [ [ 1 , 2 ] , [ 3 , 4 ] ] )
    print 'A = \n',A
    b = numpy.matrix ( [ [ -4 ] , [ 1 ] ] )
    print 'b = \n',b
    print 'A * b = \n',A*b
    x=numpy.linalg.solve(A,b)
    print'Solve Ax = b; x = \n',x

if __name__=='__main__':
    main()
