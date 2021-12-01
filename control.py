#import needed
import time
<<<<<<< Updated upstream
from Thymio import Thymio


#needed variables definition
#translation
forward_time=1
run_speed=100
#rotation
rot_time=1
rot_speed=100

#motion functions
def forward(forward_time):
=======
import numpy as np
from Thymio import Thymio

#inputs
#from global nav
next_point=(1.00,1.00)
#from filter
actual_point=(0.00,0.00)
actual_angle=0.00

#needed variables definition
#translation: speed approx 20cm/s, need to know the coefficient linking coordinates to meters
coeff_dist=1
run_speed=100
coeff_speed=21.73913043478261
#rotation
#rot_coeff=
rot_speed=100
coeff_rotspeed=1

#motion functions
def forward(next,actual):
    dist=np.sqrt(np.sum(np.square(next-actual)))
    forward_time=(dist*coeff_dist)/(run_speed*coeff_speed)
>>>>>>> Stashed changes
    th.set_var("motor.left.target",run_speed)
    th.set_var("motor.right.target",run_speed)
    time.sleep(forward_time)
    stopmotors()

<<<<<<< Updated upstream
#def turnleft():

#def turnright():
=======
#can be optimised in order to choose which rotation direction is the shortest
def turn(next,actual,actual_angle):
    direction=next-actual
    new_angle=np.atan2(direction)
    angle_diff=new_angle-actual_angle #in radians
    rot_time=(angle_diff)/(rot_speed*coeff_rotspeed)
    if(angle_diff>0):
        th.set_var("motor.left.target",2**16-rot_speed)
        th.set_var("motor.right.target",rot_speed)
    elif(angle_diff<0):
        th.set_var("motor.left.target",rot_speed)
        th.set_var("motor.right.target",2**16-rot_speed)
    time.sleep(rot_time)
    stopmotors()
>>>>>>> Stashed changes

def stopmotors():
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)