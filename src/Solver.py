#!/usr/bin/env python
import os
import time
import numpy as np
import scipy as sp
import cv2
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

np.set_printoptions(threshold='nan')

FIRST = 0

class Solver:
    
    def __init__(self,maxElements=None,lam=0.1):
        self.maxElements = maxElements
        self.lam = lam
        self.numRows = 4
        self.A1 = np.diag(np.ones((4,)))   #starting position
        self.A2 = np.diag(np.ones((4,)))   #starting position
        self.b1 = np.array([16.5,15.24, 9.0, -0.5]).reshape(-1,1)
        self.b2 = np.array([16.5, 15.24, 9.0, -0.5]).reshape(-1,1)
        
        #self.b2x = 16.5
        #self.b2y = 15.24
        #self.b2z = 9.0
        #self.b2h = -0.5

#    def setStart(self,x,y,z,h):
#        self.b1 = np.array([x, y, z, h])
#       self.b2 = np.array([x, y, z, h])
        
    def addRelative(self,dx,dy,dz,dh,num_row):
        # Grows both arrays, A2/b2 being the VO
        # A1/b1 are map odometry, initialized to 0,then later filled out by addMap
        htemp = np.zeros((4,self.numRows))
        self.numRows += 4
        vtemp = np.zeros((self.numRows,4))
        self.A1 = np.concatenate((self.A1,htemp), axis=0)
        self.A1 = np.concatenate((self.A1,vtemp), axis=1)
        self.A1[-4,-4] = 0
        self.A1[-3,-3] = 0
        self.A1[-2,-2] = 0
        self.A1[-1,-1] = 0
        
        self.b1 = np.concatenate((self.b1,np.zeros((4,1))),axis=0)
        #self.b1[-3] = 0
        #self.b1[-2] = 0
        #self.b1[-1] = 0
        
    
        self.A2 = np.concatenate((self.A2,htemp), axis=0)
        self.A2 = np.concatenate((self.A2,vtemp), axis=1)
        self.A2[-4,-4] = 1
        self.A2[-4,-8] = -1
        self.A2[-3,-3] = 1
        self.A2[-3,-7] = -1
        self.A2[-2,-2] = 1
        self.A2[-2,-6] = -1
        self.A2[-1,-1] = 1
        self.A2[-1,-5] = -1
        
        self.b2 = np.concatenate((self.b2,np.zeros((4,1))),axis=0)
        self.b2[-4] = dx
        self.b2[-3] = dy
        self.b2[-2] = dz
        self.b2[-1] = dh

        print 'Relative \n'
        print num_row
        print self.A1.shape

        #self.b2x += dx
        #self.b2y += dy
        #self.b2z += dz
        #self.b2h += dh
        #if self.b2h > math.pi:
        #    self.b2h -= 2*math.pi
        #elif self.b2h <= -math.pi:
        #    self.b2h += 2*math.pi

        
    def addMap(self, x,y,z,h,num_row):
        # Assume first num_row given is 0, and this corresponds to first non-origin entry
        # First entries changed would then be rows 5-8
        print 'Map \n'
        print num_row
        print self.A1.shape
        row = int(num_row*4)+4

        self.A1[row,row] = 1
        self.A1[row+1,row+1] = 1
        self.A1[row+2,row+2] = 1
        self.A1[row+3,row+3] = 1
        
        self.b1[row] = x
        self.b1[row+1] = y
        self.b1[row+2] = z
        self.b1[row+3] = h
        

    def SolveLSQ(self):
        min_dim = min(self.A1.shape+self.A2.shape+(self.b1.shape[0],self.b2.shape[0]))

        A = self.A1[0:min_dim,0:min_dim] \
            +self.lam*self.A2[0:min_dim,0:min_dim]

        b = self.b1[0:min_dim,0] + self.lam*self.b2[0:min_dim,0]

        x = np.linalg.solve(A,b)
        return x
        
        
    def getA1(self):
        return self.A1

def rel_callback(data):
    dx = data.linear.x
    dy = data.linear.y
    dz = data.linear.z
    dh = data.angular.z
    global FIRST
    global SOLVER
    # FIRST = 0
    if FIRST == 0:
        FIRST = data.angular.x
    
    num_row = data.angular.x - FIRST
    #print num_row
    SOLVER.addRelative(dx,dy,dz,dh,num_row)

def map_callback(data):
    x = data.linear.x
    y = data.linear.y
    z = data.linear.z
    h = data.angular.z
    num_row = data.angular.x
    global SOLVER
    global FIRST
    
    num_row = data.angular.x - FIRST
    SOLVER.addMap(x,y,z,h,num_row)
  
if __name__== "__main__":
    

    rospy.init_node('solver', anonymous=True)
    sub_rel = rospy.Subscriber("/local_pose_est", Twist, rel_callback, queue_size=5)
    sub_map = rospy.Subscriber('/corrected_pose', Twist, map_callback, queue_size=5)
    pub = rospy.Publisher("/bundle_fun", Twist, queue_size=1)
    global SOLVER
    SOLVER = Solver(50,10)
    # global FIRST
    # FIRST = 0
    r = rospy.Rate(1.0)
    msg = Twist()
    while not rospy.is_shutdown():
        
        X = SOLVER.SolveLSQ()
        x = X[-4]
        y = X[-3]
        z = X[-2]
        h = X[-1]
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = h
        pub.publish(msg)
        r.sleep()

    np.savetxt('final_state.txt',X,delimiter=',')
    np.savetxt('final_odom.txt',SOLVER.b2,delimiter=',')
    np.savetxt('final_map.txt',SOLVER.b1,delimiter=',')
    #x_correct = np.array([[30],[12],[24]])
    #for i in range(1,31):
     #   a = np.array((1+np.random.normal(0,0.02),0.4+np.random.normal(0,0.02),0.8+np.random.normal(0,0.02)))
      #  solver.addRelative(a)
       # if i % 10 == 0:
        #    b = np.array((1*i+np.random.normal(0,0.02),0.4*i+np.random.normal(0,0.02),0.8*i+np.random.normal(0,0.02)))
         #   solver.addMap(b)
    #x = solver.SolveLSQ()
    #error = np.sum((x[-3:] - x_correct)**2)
    #print error
