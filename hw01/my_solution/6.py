#!/usr/bin/env python
import numpy as np
def main():
    A = np.matrix ( [ [ 3,2] , [ 1 , -2 ] ] )
    B = np.matrix ( [ [ -1,-2] , [ 4 , -2 ] ] )
    print 'A = \n',A
    print 'B = \n',B
    print '*********** '
    print '6.a \n'
    print A+2*B
    print '*********** '
    print '6.b \n'
    print 'AB = \n', A*B
    print 'BA = \n', B*A
    print '*********** '
    print '6.c \n'
    print 'transpose of A = \n',np.transpose(A)
    print '*********** '
    print '6.d \n'
    print 'BB = \n',B*B
    print '*********** '
    print '6.e \n'
    print 'transpose of (AB) = \n',np.transpose(A*B)
    print '*********** '
    print '6.f \n'
    print 'det (A) = \n',np.linalg.det(A)
    print '*********** '
    print '6.g \n'
    print 'inverse of B = \n',np.linalg.inv(B)
    print '*********** '



if __name__=='__main__':
    main()

