from math import *
from matrix import matrix

def kalman_estimate(x, P, measurement):

    # prediction
    x = (F * x) + u
    P = F * P * F.transpose()

    # measurement update
    Z = matrix([measurement])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    return (x,P)

dt = 0.1

x=matrix([[0],[0],[0]])
#in this implementation I am using three variables in x matrix
#postion, velocity, acceleration, but measurements will only be taken for
#velocity and accelration (velocity from DVL and acceleration from both DVL and IMU)
F=matrix([[1,dt,0.5*dt*dt],[0,1,dt],[0,0,0]])     #state equation matrix
u=matrix([[0],[0],[0]])         #no raw additon to next state (for now)
H=matrix([[0,1,0],[0,0,1]])     #observation matrix (measurement modulation)
P=matrix([[0.01,0,0],[0,1,0],[0,0,1]])     #error modulation for nest step
Q=matrix([[0.01,0,0],[0,0.01,0],[0,0,1]])      #process error covariance
R=matrix([[0.01,0],[0,0.5]])                     #measurement error covariance
I=matrix([[1,0,0],[0,1,0],[0,0,1]])

measurements=[[0,1],[0.1,1],[0.2,0]]
print 'Initial conditions : '
print 'x ='
x.show()
print 'P ='
P.show()
print '------------------------------------------------------'
for i in range(1,3) :
    print 'Iteration %s :' % i
    print 'measurement : v={0}, a={1}'.format(measurements[i-1][0],measurements[i-1][1])
    (x,P)=kalman_estimate(x,P,measurements[i-1])
    print 'x ='
    x.show()
    print 'P ='
    P.show()
    print '------------------------------------------------------'
