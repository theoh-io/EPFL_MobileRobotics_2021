from tdmclient import ClientAsync, aw

def update_state(state,obstThrL,obstThrH, obst,client):
    if state == 1: 
        # switch from goal tracking to obst avoidance if obstacle detected
        if (obst[2] > obstThrH):
            state = 2
        elif (obst[1] > obstThrH):
            state = 2
        elif (obst[3] > obstThrH):
            state = 2
        elif (obst[0] > obstThrH):
            state = 2
        elif (obst[4] > obstThrH):
            state = 2
    elif state == 2:
        if obst[2] < obstThrL:
            if obst[1] < obstThrL:
                if obst[3] < obstThrL:
                    if obst[0] < obstThrL:
                        if obst[4] < obstThrL:
                            # switch from obst avoidance to goal tracking if obstacle got unseen
                            state = 1
                            aw(client.sleep(0.6))
    return state

def local_nav(prox_horizontal,y):
    
    # obstacle avoidance: ANN weights
    w_l = [40,  20, -20, -20, -40,  30, -10, 8, 0]
    w_r = [-40, -20, -20,  20,  40, -10, 30, 0, 8]

    # Scale factors that divide sensor inputs and memory inputs
    sensor_scale = 800
    memory_scale = 20

    x = [0,0,0,0,0,0,0,0,0]

    # Memory
    x[7] = y[0]//memory_scale
    x[8] = y[1]//memory_scale

    for i in range(7):
        # Get and scale inputs
        x[i] = prox_horizontal[i] // sensor_scale

    y = [0,0]    
    for i in range(len(x)):    
        # Compute outputs of neurons and set motor powers
        y[0] = y[0] + x[i] * w_l[i]
        y[1] = y[1] + x[i] * w_r[i]
    
    return y