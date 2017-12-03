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
    def __init__(self,key_pwm=9000,key_r=1000,key_theta=2000,key_phi=3000,keyX=7500,key_fwdA=6000):
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
        self.targetX 	= 0.0
        self.targetY 	= 0.0
        self.targetZ 	= 0.0

        self.distance 	= 10.0
        self.oldDistance= 10.0

        self.memoryFwdA = attach_mem(key_fwdA,4)
        self.memoryUpA 	= attach_mem(key_fwdA+200,4)
        self.memorySideA= attach_mem(key_fwdA+100,4)
        self.memoryAngA = attach_mem(key_fwdA+300,4)
        
        
        self.memoryPWM 	= load_mem(key_pwm,24)
        self.memoryR 	= load_mem(key_r,4)
        self.memoryTheta= load_mem(key_theta,4)
        self.memoryPhi 	= load_mem(key_phi,4)


        self.memoryX = load_mem(keyX,4)
        self.memoryY = load_mem(keyX+100,4)
        self.memoryZ = load_mem(keyX+200,4)
        
    def angleUpdate(self):
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
        
    def setPWM(self,pwm):	#Loads the pwm values from the array format from share mem
        
        self.side_m 	= pwm[SIDE_MOTORS:SIDE_MOTORS+2]
        self.back_m 	= pwm[BACK_MOTORS:BACK_MOTORS+2]
        self.bottom_m 	= pwm[BOTTOM_MOTORS:BOTTOM_MOTORS+2]

    def getIP_PWM(self):
        #Load IP,PWM values from shared memory
        #default keys are given, may be changed
        
       
        dat 		= self.memoryPWM.read()
        for i in range(6):		#pwm is shared as array
            pwm[i] = read_int(dat[4*i:4*i+4])
            
        self.r 		= read_float(self.memoryR.read())
        self.theta 	= read_float(self.memoryTheta.read())
        self.phi 	= read_float(self.memoryPhi.read())
        
        self.setPWM(pwm)		#to read from the array format

    
    def getTargetXYZ(self):

        self.targetX 	= read_float (self.memoryX.read())
        self.targetY	= read_float (self.memoryY.read())
        self.targetZ	= read_float (self.memoryZ.read())
        self.oldDistance = self.distance
        self.distance 	= pow(self.targetX**2+self.targetY**2+self.targetZ**2,0.5)

        
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

        
    def shareIP(self):
        # write back the updated r,theta,phi to memory
        
        write_float(self.memoryR,self.r)
        write_float(self.memoryTheta,self.theta)
        write_float(self.memoryPhi,self.phi)
        
    def update(self):
        self.getIP_PWM()
        self.setVelocity()
        self.shareVelocities()
        self.getTargetXYZ()
        self.angleUpdate()	#to be worked on
        self.shareIP()
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
