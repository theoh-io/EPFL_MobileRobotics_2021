"""
TODO: calibrate the coefficient for translation speed and rot
test the simple controller
switch to PID/ASTOLFI
"""

#import needed
import numpy as np
from tdmclient import ClientAsync


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

#low level functions
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }


#middle level function
def forward(next,actual):
    direction=np.subtract(next,actual)
    dist=np.sqrt(np.sum(np.square(direction)))
    forward_time=(dist*coeff_dist)/(run_speed*coeff_speed)
    aw(node.set_variables(motors(run_speed,run_speed)))
    client.sleep(forward_time)
    stopmotors()


def turn(next,actual,actual_angle):
    global node
    #should we use the circular notation for negative: 2**16-??
    direction=np.subtract(next,actual)
    new_angle=np.arctan2(direction[0],direction[1])
    """"
    if((direction[0])and(not direction[1])):
        new_angle=np.arctan2(direction,(0,0))[0]
    elif((not direction[0])and(not direction[1])):
        new_angle=np.arctan2(direction,(0,0))[1]
    elif(direction[0] and direction[1]):
        new_angle=np.arctan2(direction,(0,0))[2]
    elif((not direction[0])and(direction[1])):
        new_angle=np.arctan2(direction,(0,0))[3]
    """
    angle_diff=np.degrees(new_angle-actual_angle) #in radians
    rot_time=(angle_diff)/(rot_speed*coeff_rotspeed)
    if(angle_diff>0):
        aw(node.set_variables(motors(-rot_speed,rot_speed)))
        client.sleep(rot_time)
    elif(angle_diff<0):
        aw(node.set_variables(motors(rot_speed,-rot_speed)))
        client.sleep(rot_time)
    stopmotors()

def stopmotors():
    global node
    aw(node.set_variables(motors(0,0)))