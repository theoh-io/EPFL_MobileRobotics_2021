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
#temps pour faire 1 tour: 9,5s => vitesse angulaire=360/9,5
rot_motor_speed=100
rot_real_speed=37.895

#coefficients for the Astolfi controller
r=0.02 #need to mesure more precisely
l=0.04 #need to be measured more precisely
kp=2 #>0
ka=0.2  #must be > kp
kb=-0.1 #<0


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

def angle2points(next, actual, node):
    direction=np.subtract(next,actual)
    new_angle=np.degrees(np.arctan2(direction[1],direction[0]))
    return new_angle

#def read_globalframe():
#can be useful to pass to filtering

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
    new_angle=np.degrees(np.arctan2(direction[1],direction[0])) #first argument is y !!
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
    angle_diff=new_angle-actual_angle 
    rot_time=(abs(angle_diff))/(rot_real_speed)
    if(angle_diff>0):
        set_motors(-rot_motor_speed,rot_motor_speed, node)
        aw(client.sleep(rot_time))
    elif(angle_diff<0):
        set_motors(rot_motor_speed,-rot_motor_speed, node)
        aw(client.sleep(rot_time))
    stopmotors(node)


def navigate(next,actual,actual_angle, node, client):
    turn(next,actual,actual_angle, node, client)
    forward(next,actual, node, client)

#calibration: just rotate on itself
def calib_rot(node,client):
    aw(node.set_variables(motors(-rot_motor_speed,rot_motor_speed)))
 

def astolfi(actual_pos, goal_pos, actual_angle, node):
    delta=np.subtract(goal_pos,actual_pos)
    pho=np.sqrt(np.sum(np.square(delta)))
    alpha=-actual_angle + np.degrees(np.arctan2(delta[1],delta[0]))
    beta=-actual_angle-alpha
    v=kp*pho
    omega=ka*alpha+kb*beta
    right_speed=(l*omega+v)/r
    left_speed=(v-l*omega)/r
    left_speed=int(left_speed)
    print(left_speed)
    right_speed=int(right_speed)
    set_motors(left_speed, right_speed, node)
    
