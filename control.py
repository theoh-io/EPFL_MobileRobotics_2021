"""
TODO: calibrate the coefficient for translation speed and rot
test the simple controller
switch to PID/ASTOLFI
"""

#import needed
import numpy as np
import time
from tdmclient import ClientAsync, aw


#needed variables definition
#motors speed no unit
motor_speed=100
#translation: speed approx 3,25 [cm/s]
real_speed=3.25   #21.73913043478261
#coeff linking thymio dist to real [coord/cm] (suppose 1 coord = 20cm)
dist_coord=10 # [cm/coord] coord thymio ref = 10cm
coeff_dist=dist_coord


#rotation
#temps pour faire 1 tour: 10s => vitesse angulaire=360/10
rot_motor_speed=100
rot_real_speed=36

#motion functions

#low level functions
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

#middle level functions
def set_motors(left,right,node):
    aw(node.set_variables(motors(left,right)))

def stopmotors(node):
    aw(node.set_variables(motors(0,0)))

def read_motors_speed(node,client):
    aw(node.wait_for_variables({"motor.left.speed","motor.right.speed"}))
    aw(client.sleep(0.3))
    speed=[node.v.motor.left.speed, node.v.motor.right.speed]
    return speed


#high level function
def forward(next,actual,node, client):
    direction=np.subtract(next,actual)
    dist=np.sqrt(np.sum(np.square(direction)))
    #dist: coord, coeff_dist: cm/coord, motor_speed[motor], coeff speed cm/s at 100
    #print(dist, coeff_dist, motor_speed, real_speed)
    forward_time=(dist*coeff_dist)/(real_speed) 
    #print(forward_time)
    set_motors(motor_speed,motor_speed,node)
    aw(client.sleep(forward_time))
    stopmotors(node)


def turn(next,actual,actual_angle, node, client):
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
    rot_time=(angle_diff)/(rot_real_speed)
    if(angle_diff>0):
        aw(node.set_variables(motors(-rot_motor_speed,rot_motor_speed)))
        aw(client.sleep(rot_time))
    elif(angle_diff<0):
        aw(node.set_variables(motors(rot_motor_speed,-rot_motor_speed)))
        aw(client.sleep(rot_time))
    stopmotors(node)

def navigate(next,actual,actual_angle, node, client):
    turn(next,actual,actual_angle, node, client)
    forward(next,actual, node, client)

#calibration: just rotate on itself
def calib_rot(node,client):
    aw(node.set_variables(motors(-rot_motor_speed,rot_motor_speed)))
 

    
