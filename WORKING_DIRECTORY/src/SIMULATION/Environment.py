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
    def __init__(self,keyX=7500,keyFwdA=6000):
        
        self.X = 0.0		#coordinates
        self.Y = 0.0
        self.Z = 0.0
        self.Orientation = 0.0  #angle with x
        
        self.forward_a 	= 0.0	#velocities
        self.side_a 	= 0.0
        self.upward_a 	= 0.0
        self.angular_a 	= 0.0
        
        self.targets=[]

        self.memoryX 	= attach_mem(keyX,4)
        self.memoryY 	= attach_mem(keyX+100,4)
        self.memoryZ 	= attach_mem(keyX+200,4)
        self.memoryID 	= attach_mem(keyX+300,4)

        self.memoryFwdA	= load_mem(keyFwdA,4)
        self.memoryUpA 	= load_mem(keyFwdA+100,4)
        self.memorySideA= load_mem(keyFwdA+200,4)
        self.memoryAngA = load_mem(keyFwdA+300,4)
    
        
        
    def addTarget(self,k):
        #add a target to the list of targets 
        #k is a tuple (x,y,z,taskID)
        
        self.targets.append(k)

    def shareTargetXYZ(self,index=0,key=7500):
        # share relative positions of target and taskID

        write_float(self.memoryX,self.targets[index][0]-self.X)
        write_float(self.memoryY,self.targets[index][1]-self.Y)
        write_float(self.memoryZ,self.targets[index][2]-self.Z)
        self.memoryID.write(self.targets[index][3])
        
    def getVelocities(self):
        #load from shared memory
        
        self.forward_a 	= read_float (self.memoryFwdA.read())
        self.side_a 	= read_float (self.memorySideA.read())
        self.up_a 	= read_float (self.memoryUpA.read())
        self.angular_a 	= read_float (self.memoryAngA.read())


    def distanceUpdate(self):
        #change coordinates according to velocity
        
        self.Orientation += self.angular_a * TIME_STEP * THETA_FACTOR

        delX = self.forward_a * math.cos(self.Orientation) + self.side_a * math.sin(self.Orientation)
        self.X += delX*LINEAR_FACTOR*TIME_STEP

        delY = self.forward_a * math.sin(self.Orientation) + self.side_a * math.cos(self.Orientation)
        self.Y += delY*LINEAR_FACTOR*TIME_STEP

        self.Z += self.upward_a * TIME_STEP *LINEAR_FACTOR

        
    def update(self):
        
        self.getVelocities()
        self.distanceUpdate()
        self.shareTargetXYZ()
        time.sleep(0.1)


def main():
    e = Environment()
    e.addTarget((10,5,5,'p'))
    i=0
    while True:
        e.update()
        print "target - ", e.targets[0]
        print "auv - ",e.X,e.Y,e.Z
        print i
        i+=1
main()
