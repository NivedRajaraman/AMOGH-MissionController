from pyread import *
import math

TIME_STEP = 0.1


class Environment:
    def __init__(self):
        
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.Orientation = 0.0    # angle with x
        
        self.forward_a = 0.0
        self.side_a = 0.0
        self.upward_a = 0.0
        self.angular_a =0.0

    def load_memory(self,key_fwdA=6000,key_sideA=6100,key_upA=6200,key_angA=6300):
        memory1 = load_mem(key_fwdA,4)
        memory2 = load_mem(key_upA,4)
        memory3 = load_mem(key_sideA,4)
        memory4 = load_mem(key_angA,4)
    
        self.forward_a 	= read_float (memory1.read())
        self.side_a 	= read_float (memory2.read())
        self.up_a 	= read_float (memory3.read())
        self.angular_a 	= read_float (memory4.read())


    def distance_update(self):
        self.Orientation += self.angular_a * TIME_STEP
        self.X += self.forward_a * TIME_STEP * math.cos(self.Orientation) + self.side_a * TIME_STEP * math.sin(self.Orientation)
        self.Y += self.forward_a * TIME_STEP * math.sin(self.Orientation) + self.side_a * TIME_STEP * math.cos(self.Orientation)
        self.Z += self.upward_a * TIME_STEP

        
    def update(self):

        self.load_memory()
        self.distance_update()
