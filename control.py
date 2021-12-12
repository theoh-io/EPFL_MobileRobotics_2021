#import needed
import numpy as np
from tdmclient import ClientAsync, aw



#coefficients for the Astolfi controller 
#thymio measures (mm)
r=22
l=48 
#coefficients of the controller and threshold of it's FSM
kp=25   #>0
ka=50  # > kp
kb=-0.0001 #<0
thresh_far=300
thresh_close2=10
thresh_close1=75
comm_sat_min=100
comm_sat_max=250


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


#astolfi controller
#astolfi return switch_next is the transition condition of it's FSM to iterate on the next goal from the path list
def astolfi(actual_pos, goal_pos, actual_angle, node): 
    switch_next=0
    delta=np.subtract(goal_pos,actual_pos)
    pho=np.sqrt(np.sum(np.square(delta)))
    alpha=-actual_angle + np.arctan2(-delta[1],delta[0])
    beta=-actual_angle-alpha
    v=kp*pho
    if(pho>thresh_far):
        v=comm_sat_max*r
    if(pho<thresh_close1):
        v=comm_sat_min*r
    if(pho<thresh_close2):
        v=0
        switch_next=1      
    omega=ka*alpha+kb*beta
    right_speed=(l*omega+v)/r
    left_speed=(v-l*omega)/r
    left_speed=int(left_speed)
    right_speed=int(right_speed)
    set_motors(left_speed, right_speed, node)
    return switch_next
   
    
