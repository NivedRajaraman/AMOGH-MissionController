"""

Work in progress 
Possibly to be abandoned

Simulator code to test shared memory and main.cpp in general

"""

from pyread import *


pwm=[0]*6

LINEAR_FACTOR	= 0.005   #linear scaling factor
SIDE_MOTORS 	= 0  ##0-1 side
BACK_MOTORS 	= 2  ##2-3 back
BOTTOM_MOTORS 	= 4 ##3-4 bottom
THRESHOLD 	= 1500
HOVER_VALUE 	= 0
THETA_FACTOR	= 0.005   #linear scaling factor
PHI_FACTOR	= 0.005   #linear scaling factor
TIME_STEP 	= 0.1

class AUV:
    def __init__(self):
        self.r = 0.0
        self.theta = 0.0
        self.phi = 0.0
        self.side_m = 0.0
        self.back_m = 0.0
        self.bottom_m = 0.0
        self.forward_a = 0.0
        self.side_a = 0.0
        self.upward_a = 0.0
        self.theta_angular_a = 0.0
        
    def angle_update(self,distance = 5): #### distance ?
        r = r + r * 1/(1-(self.forward_a*LINEAR_FACTOR)*TIME_STEP/distance )   #### distance * angle ( = r) = constant
        
        
    def set_pwm(self,pwm):
        self.side_m = pwm[SIDE_MOTORS:SIDE_MOTORS+2]
        self.back_m = pwm[BACK_MOTORS:BACK_MOTORS+2]
        self.bottom_m = pwm[BOTTOM_MOTORS:BOTTOM_MOTORS+2]

    def load_memory(self,key_pwm=9000,key_r=1000,key_theta=2000,key_phi=3000):

        memory0 = load_mem(key_pwm,24)
        memory1 = load_mem(key_r,4)
        memory2 = load_mem(key_theta,4)
        memory3 = load_mem(key_phi,4)
        dat 	= memory0.read()
        
        for i in range(6):
            pwm[i] = read_int(dat[4*i:4*i+4])

        self.r = read_float(memory1.read())
        self.theta = read_float(memory2.read())
        self.phi = read_float(memory3.read())
        self.set_pwm(pwm)

    def update(self):
        self.load_memory()
        self.forward_a         	= (self.back_m[0] + self.back_m[1])/2.0-THRESHOLD # forward acceleration
        self.side_a    		= (self.side_m[1] - self.side_m[0]) #assume right is positive
        self.upward_a 		= (bottom_m[0]+bottom_m[1])/2.0-THRESHOLD - HOVER_VALUE #Needs some value to maintain depth
        self.theta_angular_a 	= (back_m[1]-back_m[0]) #rotating in the horizontal plane
        
        self.angle_update()
        
        time.sleep(0.1)

        
    def write_memory(self,key_pwm=9000,key_r=1000,key_theta=2000,key_phi=3000):
        
        memory1 = load_mem(key_r,4)
        memory2 = load_mem(key_theta,4)
        memory3 = load_mem(key_phi,4)
        write_mem(memory1,self.r)
        write_mem(memory2,self.theta)
        write_mem(memory3,self.phi)
