import tdmclient.notebook
await tdmclient.notebook.start()

%%run_python 

y = [0,0]        # powers given to motor wheels with global nav, here TEST 0,0
state = 0          # 0=global path, 1=obstacle avoidance
obst = [0,0,0]     # measurements from left middle right prox sensors
    
timer_period[0] = 10   # 10ms sampling time
obstThrL = 10      # low obstacle threshold to switch state 1->0
obstThrH = 20      # high obstacle threshold to switch state 0->1  

@onevent
def timer0():
    global prox_horizontal, motor_left_target, motor_right_target, button_center, state, y, obst, obstThrH, obstThrL, obstSpeedGain, speed0, speedGain 

    # acquisition from the proximity sensors to detect obstacles
    obst = [prox_horizontal[0], prox_horizontal[2], prox_horizontal[4]]
    
    # tdmclient does not support yet multiple and/or in if statements:
    if button_center == 1:
        state = 1 if state==0 else 0
    
    if state == 0: 
        # switch from goal tracking to obst avoidance if obstacle detected
        if (obst[2] > obstThrH):
            state = 1
            if (obst[0] > obstThrH):
                state = 1
            elif (obst[1] > obstThrH):
                state = 1
    elif state == 1:
        if obst[2] < obstThrL:
            if obst[0] < obstThrL:
                if obst[1] < obstThrL : 
                # switch from obst avoidance to goal tracking if obstacle got unseen
                state = 0
                
    if  state == 0 :
        # goal tracking: go according to global path with y
        leds_top = [0,0,0]
        y[0]=0
        y[1]=0
        
    else:
        leds_top = [30,30,30]
        # obstacle avoidance: ANN
        w_l = [40,  20, -20, -20, -40,  30, -10, 8, 0]
        w_r = [-40, -20, -20,  20,  40, -10, 30, 0, 8]

        # Scale factors for sensors and constant factor
        sensor_scale = 200
        constant_scale = 20
    
        x = [0,0,0,0,0,0,0,0,0]
        x[7] = y[0]//5
        x[8] = y[1]//5
        
        for i in range(7):
            # Get and scale inputs
            x[i] = prox_horizontal[i] // sensor_scale
        
        y = [0,0]    
        for i in range(len(x)):    
            # Compute outputs of neurons and set motor powers
            y[0] = y[0] + x[i] * w_l[i]
            y[1] = y[1] + x[i] * w_r[i]
    
    # Set motor powers
    motor_left_target = y[0]
    motor_right_target = y[1]

await tdmclient.notebook.stop()