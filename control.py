#import needed
import time
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
    th.set_var("motor.left.target",run_speed)
    th.set_var("motor.right.target",run_speed)
    time.sleep(forward_time)
    stopmotors()

#def turnleft():

#def turnright():

def stopmotors():
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)