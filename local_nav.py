def update_state(state,obstThrL,obstThrH):
    if state == 0: 
        # switch from goal tracking to obst avoidance if obstacle detected
        if (obst[0] > obstThrH):
            state = 1
            print("change state to local")
        elif (obst[2] > obstThrH):
            state = 1
            print("change state to local")
        elif (obst[1] > obstThrH):
            state = 1
            print("change state to local")
    elif state == 1:
        if obst[0] < obstThrL:
            if obst[2] < obstThrL:
                # switch from obst avoidance to goal tracking if obstacle got unseen
                state = 0
                print("change state to global")
    
    return state

def local_nav(prox_horizontal,y):
    leds_top = [30,30,30]
    
    # obstacle avoidance: ANN
    w_l = [40,  20, -20, -20, -40,  30, -10, 8, 0]
    w_r = [-40, -20, -20,  20,  40, -10, 30, 0, 8]

    # Scale factors for sensors and constant factor
    sensor_scale = 800
    constant_scale = 20

    x = [0,0,0,0,0,0,0,0,0]

    # Memory
    x[7] = y[0]//20
    x[8] = y[1]//20

    for i in range(7):
        # Get and scale inputs
        x[i] = prox_horizontal[i] // sensor_scale

    y = [0,0]    
    for i in range(len(x)):    
        # Compute outputs of neurons and set motor powers
        y[0] = y[0] + x[i] * w_l[i]
        y[1] = y[1] + x[i] * w_r[i]
    
    return y