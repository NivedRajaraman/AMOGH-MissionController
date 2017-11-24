"""

Straightforward, contains details about the AUV's location, 
and the locations of the targets. Shares these with Simulator.py.
Pretty much done.

Bug with running out of memory addresses if running without the 
time.sleep() or if letting it run for >5ish minutes - worth fixing ?

"""

from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from PYTHON_INTERFACE.pyread import * 	#For shm functions
from parameters import *

import math



class Environment:
    def __init__(self):
        
        self.X = 0.0		#coordinates
        self.Y = 0.0
        self.Z = 0.0
        self.Orientation = 0.0  #angle with x
        
        self.forward_a 	= 0.0	#velocities
        self.side_a 	= 0.0
        self.upward_a 	= 0.0
        self.angular_a 	= 0.0

        self.targets=[]


    def addTarget(self,k):
        #add a target to the list of targets 
        #k is a tuple (x,y,z,taskID)
        
        self.targets.append(k)

    def shareTargetXYZ(self,index=0,key=7500):
        # share relative positions of target and taskID
        # problem: memory leak ?
        try:
            memory = load_mem(key,4)
        except:
            memory = attach(key,4)
        write_mem(memory,self.targets[index][0]-self.X)
        """
        memory=attach(key+100,4)
        write_mem(memory,self.targets[index][1]-self.Y)

        memory=attach(key+200,4)
        write_mem(memory,self.targets[index][2]-self.Z)

        memory=attach(key+300,1)
        memory.write(self.targets[index][3])
        """     
    def getVelocities(self,key_fwdA=6000,key_sideA=6100,key_upA=6200,key_angA=6300):
        #load from shared memory
        
        memory1 = load_mem(key_fwdA,4)
        memory2 = load_mem(key_upA,4)
        memory3 = load_mem(key_sideA,4)
        memory4 = load_mem(key_angA,4)
    
        self.forward_a 	= read_float (memory1.read())
        self.side_a 	= read_float (memory2.read())
        self.up_a 	= read_float (memory3.read())
        self.angular_a 	= read_float (memory4.read())


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
        #time.sleep(0.1)


def main():
    e = Environment()
    e.addTarget((1,2,3,'p'))
    i=0
    while True:
        e.update()
        print i
        i=i+1

main()
