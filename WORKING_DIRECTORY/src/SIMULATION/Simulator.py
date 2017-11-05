"""

Work in progress 

Simulator code to test shared memory and main.cpp in general

"""

from pyread import *		#For shm functions


pwm=[0]*6

LINEAR_FACTOR	= 0.005   	#Linear scaling factor
SIDE_MOTORS 	= 0  		#0-1 side
BACK_MOTORS 	= 2  		#2-3 back
BOTTOM_MOTORS 	= 4 		#3-4 bottom
THRESHOLD 	= 1500
HOVER_VALUE 	= 0
THETA_FACTOR	= 0.005   	#Linear scaling factor
PHI_FACTOR	= 0.005   	#Linear scaling factor
TIME_STEP 	= 0.1

class AUV:
    def __init__(self):
        self.r = 0.0
        self.theta = 0.0
        self.phi = 0.0
        self.side_m = [0,0]	#motor values
        self.back_m = [0,0]
        self.bottom_m = [0,0]	
        self.forward_a = 0.0
        self.side_a = 0.0	#All the velocities
        self.upward_a = 0.0
        self.angular_a = 0.0
        
    def angle_update(self,distance = 5): 	#### distance ?
        r = r + r * 1/(1-(self.forward_a*LINEAR_FACTOR)*TIME_STEP/distance )   #### distance * angle ( = r) = constant
        
        
    def set_pwm(self,pwm):	#Loads the pwm values from the array format from share mem
        self.side_m = pwm[SIDE_MOTORS:SIDE_MOTORS+2]
        self.back_m = pwm[BACK_MOTORS:BACK_MOTORS+2]
        self.bottom_m = pwm[BOTTOM_MOTORS:BOTTOM_MOTORS+2]

    def load_memory(self,key_pwm=9000,key_r=1000,key_theta=2000,key_phi=3000):
        #Load values from shared memory
        #default keys are given, may be changed
        
        memory0 = load_mem(key_pwm,24)
        memory1 = load_mem(key_r,4)
        memory2 = load_mem(key_theta,4)
        memory3 = load_mem(key_phi,4)
        dat 	= memory0.read()
        
        for i in range(6):	#pwm is shared as array
            pwm[i] = read_int(dat[4*i:4*i+4])

        self.r = read_float(memory1.read())
        self.theta = read_float(memory2.read())
        self.phi = read_float(memory3.read())
        self.set_pwm(pwm)	#to read from the array format

    def update(self):
        self.load_memory()
        self.forward_a         	= (self.back_m[0] + self.back_m[1])/2.0-THRESHOLD 	# forward acceleration
        self.side_a    		= (self.side_m[1] - self.side_m[0]) 			#assume right is positive
        self.upward_a 		= (bottom_m[0]+bottom_m[1])/2.0-THRESHOLD - HOVER_VALUE #Needs some value to maintain depth
        self.angular_a 		= (back_m[1]-back_m[0]) 				#rotating in the horizontal plane
        
        self.angle_update()	#to be worked on
        
        time.sleep(0.1)


    def share_memory(self,key_fwdA=6000,key_sideA=6100,key_upA=6200,key_angA=6300):
        #share the values with environment.py
        
        memory1 = load_mem(key_fwdA,4)
        memory2 = load_mem(key_upA,4)
        memory3 = load_mem(key_sideA,4)
        memory4 = load_mem(key_angA,4)
        memory1.write(self.forward_a)
        memory2.write(self.upward_a)
        memory3.write(self.side_a)
        memory4.write(self.angular_a)

        
    def write_memory(self,key_pwm=9000,key_r=1000,key_theta=2000,key_phi=3000):
        # write back the updated r,theta,phi to memory
        
        memory1 = load_mem(key_r,4)
        memory2 = load_mem(key_theta,4)
        memory3 = load_mem(key_phi,4)
        write_mem(memory1,self.r)
        write_mem(memory2,self.theta)
        write_mem(memory3,self.phi)
