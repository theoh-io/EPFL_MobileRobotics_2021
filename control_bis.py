"""
TODO: calibrate the coefficient for translation speed and rot
test the simple controller
switch to PID/ASTOLFI
"""

#import needed
import numpy as np
from tdmclient import ClientAsync, aw


#needed variables definition
#motors speed no unit
motor_speed=50
#translation: speed approx 3,25 [cm/s]
real_speed=8.8   #21.73913043478261
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
    aw(client.sleep(0.01))
    speed=[node.v.motor.left.speed, node.v.motor.right.speed]
    return speed

def read_prox_sensors(node,client):
    aw(node.wait_for_variables({"prox.horizontal"}))
    aw(client.sleep(0.01))
    prox=node.v.prox.horizontal
    return prox

def angle2points(next, actual, node):
    direction=np.subtract(next,actual)
    new_angle=np.degrees(np.arctan2(direction[1],direction[0]))
    return new_angle

#def read_globalframe():
#can be useful to pass to filtering

#high level function
def forward(next,actual,node, client):
    y=[0,0]
    direction=np.subtract(next,actual)
    dist=np.sqrt(np.sum(np.square(direction)))
    #dist: coord, coeff_dist: cm/coord, motor_speed[motor], coeff speed cm/s at 100
    #print(dist, coeff_dist, motor_speed, real_speed)
    forward_time=(dist*coeff_dist)/(real_speed) 
    #print(forward_time)
    y=[motor_speed,motor_speed]
    
    return y


def turn(next,actual,actual_angle, node, client):
    y=[0,0]
    #should we use the circular notation for negative: 2**16-??
    new_angle=angle2points(next, actual, node)#first argument is y !!
    angle_diff=new_angle-actual_angle 
    rot_time=(abs(angle_diff))/(rot_real_speed)
    if(angle_diff>0):
        y=[-rot_motor_speed,rot_motor_speed]
    elif(angle_diff<0):
        y=[rot_motor_speed,-rot_motor_speed]
        
    return y


def navigate(next,actual,actual_angle, node, client):
    y_turn = turn(next,actual,actual_angle, node, client)
    y_forward = forward(next,actual, node, client)
    
    return y_turn,y_forward

def globnav(checkpoints,starting_angle,node,client):
    angle=starting_angle
    for i in range(len(checkpoints)-1):   
        [y_turn,y_forward]=navigate(checkpoints[i+1], checkpoints[i], angle, node, client)
        angle=angle2points(checkpoints[i+1],checkpoints[i],node)


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
    right_speed=int(right_speed)
    y=left_speed,right_speed
    
    return y
    
    

