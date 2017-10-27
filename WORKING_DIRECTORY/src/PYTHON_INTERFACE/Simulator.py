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

def angle_update(r,del_theta,del_phi):
    del_theta *= 3.14/180.0  #convert to radian
    del_phi *= 3.14/180.0
    r = r - del_theta*r
    #whattodohere
    return r,del_theta,del_phi
    
while 1:
    memory0 = load_mem(9000,24)
    memory1 = load_mem(1000,4)
    memory2 = load_mem(2000,4)
    memory3 = load_mem(3000,4)
    dat= memory0.read()
    for i in range(6):
        pwm[i]= read_int(dat[4*i:4*i+4])

    side_m = pwm[SIDE_MOTORS:SIDE_MOTORS+2]
    back_m = pwm[BACK_MOTORS:BACK_MOTORS+2]
    bottom_m = pwm[BOTTOM_MOTORS:BOTTOM_MOTORS+2]
    r = read_float(memory1.read())
    theta = read_float(memory2.read())
    phi = read_float(memory3.read())
    print pwm
    print r,theta,phi
    
    forward_a  		= (back_m[0]+back_m[1])/2.0-THRESHOLD # forward acceleration
    side_a		= (side_m[1]-side_m[0]) #assume right is positive
    theta_angular_a 	= (back_m[1]-back_m[0]) #rotating in the horizontal plane
    upward_a 		= (bottom_m[0]+bottom_m[1])/2.0-THRESHOLD - HOVER_VALUE #Needs some value to maintain depth

    r 			= r-forward_a *LINEAR_FACTOR
    phi_angular_a 	= (back_m[1]-back_m[0])
    del_theta		= theta_angular_a*THETA_FACTOR
    del_phi 		= phi_angular_a*PHI_FACTOR
    
    r,del_theta,del_phi = angle_update(r,del_theta,del_phi)

    phi = phi +del_phi
    theta = theta + del_theta

    write_mem(memory1,r)
    write_mem(memory2,theta)
    write_mem(memory3,phi)
    time.sleep(0.1)
