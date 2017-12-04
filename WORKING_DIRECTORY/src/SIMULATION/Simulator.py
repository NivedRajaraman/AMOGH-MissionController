"""

Work in progress 

Simulator code to test shared memory and main.cpp in general

"""
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from PYTHON_INTERFACE.pyread import *	#For shm functions
from parameters import *


pwm=[0]*6

class AUV:
    def __init__(self,key_pwm=9000,keyX=7500,key_fwdA=6000):
        self.r 		= 0.0
        self.theta 	= 0.0
        self.phi 	= 0.0
        self.side_m 	= [0,0]	#motor values
        self.back_m 	= [0,0]
        self.bottom_m 	= [0,0]	
        self.forward_a 	= 0.0
        self.side_a 	= 0.0	#All the velocities
        self.upward_a 	= 0.0
        self.angular_a 	= 0.0

        self.memoryFwdA = attach_mem(key_fwdA,4)
        self.memoryUpA 	= attach_mem(key_fwdA+200,4)
        self.memorySideA= attach_mem(key_fwdA+100,4)
        self.memoryAngA = attach_mem(key_fwdA+300,4)

        self.memoryPWM 	= load_mem(key_pwm,24)
            
   def setPWM(self,pwm):	#Loads the pwm values from the array format from share mem
        
        self.side_m 	= pwm[SIDE_MOTORS:SIDE_MOTORS+2]
        self.back_m 	= pwm[BACK_MOTORS:BACK_MOTORS+2]
        self.bottom_m 	= pwm[BOTTOM_MOTORS:BOTTOM_MOTORS+2]

    def setVelocity(self):
        self.forward_a         	= (self.back_m[0] + self.back_m[1])/2.0-THRESHOLD 	# forward acceleration
        self.side_a    		= (self.side_m[1] - self.side_m[0]) 			# assume right is positive
        self.upward_a 		= (self.bottom_m[0]+self.bottom_m[1])/2.0-THRESHOLD - HOVER_VALUE # Needs some value to maintain depth
        self.angular_a 		= (self.back_m[1]-self.back_m[0]) 				# rotating in the horizontal plane
        
        
    def shareVelocities(self):
        #share the values with environment.py
        
        write_float(self.memoryFwdA,self.forward_a)
        write_float(self.memoryUpA,self.upward_a)
        write_float(self.memorySideA,self.side_a)
        write_float(self.memoryAngA,self.angular_a)


    def getPWM():
        dat 		= self.memoryPWM.read()
        for i in range(6):		#pwm is shared as array
            pwm[i] = read_int(dat[4*i:4*i+4])
            
        self.setPWM(pwm)		#to read from the array format
        
    def update(self):
        self.getPWM()
        self.setVelocity()
        self.shareVelocities()
        time.sleep(0.1)


def main():
    test = AUV()
    i=0
    while True:
        test.update()
        print "rtp", test.r,test.theta,test.phi
        print "vel", test.forward_a,test.side_a,test.upward_a,test.angular_a
        print "dist",test.distance
        #print i
        i=i+1
main()
