"""

Straightforward, contains details about the AUV's location, 
and the locations of the targets. Shares these with Simulator.py.
Pretty much done.

"""

from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from PYTHON_INTERFACE.pyread import * 	#For shm functions
from parameters import *

import math



class Environment:
    def __init__(self,keyX=7500,keyFwdA=6000,key_r=1000,key_theta=2000,key_phi=3000):
        
        self.X = 0.0		#coordinates
        self.Y = 0.0
        self.Z = 0.0
        self.Orientation = 0.0  #angle with x
        
        self.forward_a 	= 0.0	#velocities
        self.side_a 	= 0.0
        self.upward_a 	= 0.0
        self.angular_a 	= 0.0
        
        self.targets=[]

        self.memoryFwdA	= load_mem(keyFwdA,4)
        self.memoryUpA 	= load_mem(keyFwdA+100,4)
        self.memorySideA= load_mem(keyFwdA+200,4)
        self.memoryAngA = load_mem(keyFwdA+300,4)

        self.memoryR 	= load_mem(key_r,4)
        self.memoryTheta= load_mem(key_theta,4)
        self.memoryPhi 	= load_mem(key_phi,4)

    
        self.distance 	= 10.0
        self.oldDistance= 10.0

        self.presentTargetX	= 0.0
        self.presentTargetY	= 0.0
        self.presentTargetZ	= 0.0
        self.presentTaskID	= 'n'

        
    def getIP(self):
        #Load IP,PWM values from shared memory
        #default keys are given, may be changed
        
       
        self.r 		= read_float(self.memoryR.read())
        self.theta 	= read_float(self.memoryTheta.read())
        self.phi 	= read_float(self.memoryPhi.read())
        
        

    def shareIP(self):
        # write back the updated r,theta,phi to memory
        
        write_float(self.memoryR,self.r)
        write_float(self.memoryTheta,self.theta)
        write_float(self.memoryPhi,self.phi)
        
        
    def angleUpdate(self):
        if self.presentTaskID == 'b':
            self.angleUpdateBuoy()
        elif self.presentTaskID == 'l':
            self.angleUpdateBuoy()
        
    def angleUpdateBuoy(self):
        """
        The r seen by the camera is a function of the tangent of the angle 
        made by the centre of the object and the AUV's plane
        
        r = k*  h/x
        

        """        
        delX 	= self.distance-self.oldDistance
        delR 	= -1.0*self.r*delX/self.distance

        self.r += delR
        self.theta 	+= self.angular_a*TIME_STEP*THETA_FACTOR
        self.phi   	+= self.angular_a*TIME_STEP*THETA_FACTOR   # Theta and phi change by same amount
        

        
    def getVelocities(self):
        #load from shared memory
        
        self.forward_a 	= read_float (self.memoryFwdA.read())
        self.side_a 	= read_float (self.memorySideA.read())
        self.up_a 	= read_float (self.memoryUpA.read())
        self.angular_a 	= read_float (self.memoryAngA.read())


    def distanceUpdate(self):
        #change coordinates according to velocity

        
        self.oldDistance=self.distance

        self.Orientation += self.angular_a * TIME_STEP * THETA_FACTOR

        delX = self.forward_a * math.cos(self.Orientation) + self.side_a * math.sin(self.Orientation)
        self.X += delX*LINEAR_FACTOR*TIME_STEP

        delY = self.forward_a * math.sin(self.Orientation) + self.side_a * math.cos(self.Orientation)
        self.Y += delY*LINEAR_FACTOR*TIME_STEP

        self.Z += self.upward_a * TIME_STEP *LINEAR_FACTOR

        self.distance 	= pow((self.presentTargetX-self.X)**2+(self.presentTargetY-self.Y)**2+(self.presentTargetZ-self.Z)**2,0.5)
        self.distanceCheck()
        

    def distanceCheck(self):
        if self.distance<= DISTANCE_TOLERANCE:
            self.targets.pop()
            self.targetSet()

            
    def targetSet(self):
        if len(self.target)==0:
            self.presentTargetX= 0.0
            self.presentTargetY= 0.0
            self.presentTargetZ= 0.0
            self.presentTaskID	= 'n'
            return
        
        target	= self.targets[0]
        task 	= target[4]

        if task=='b':
            self.presentTargetX = target[0]
            self.presentTargetY = target[1]
            self.presentTargetZ = target[2]
            self.presentTaskID  = target[3]

        if task=='l':
            self.presentTargetX = (target[0][0]+target[0][1])/2
            self.presentTargetY = (target[1][0]+target[1][1])/2
            self.presentTargetZ = (target[2][0]+target[2][1])/2
            self.presentTaskID  = (target[3][0]+target[3][1])/2
            

            
    def addBuoyTarget(self,X,Y,Z):
        self.targets.append((X,Y,Z,'b'))	#Tuple of three coordinates and taskID
        if self.presentTaskID=='n':
            self.targetSet()

    def addLaneTarget(self,X1,Y1,Z1,X2,Y2,Z2):
        self.targets.append((X1,X2),(Y1,Y2),(Z1,Z2),'l') 	#X1,Y1,Z1 and X2,Y2,Z2 are the lane ends
        
            
    def update(self):
        self.getIP()
        self.getVelocities()
        self.distanceUpdate()
        self.angleUpdate()
        self.shareIP()
        time.sleep(0.1)



def main():
    e = Environment()
    e.addBuoyTarget(10,5,5)
    i=0
    while True:
        e.update()
        print "target - ", e.targets[0]
        print "auv - ",e.X,e.Y,e.Z
        print i
        i+=1
main()
