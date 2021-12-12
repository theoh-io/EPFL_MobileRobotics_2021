import time
from EKF_class import ExtendedKalmanFilterAstolfi
import numpy as np

def kalman_step(sensor_package, KF):
    #DIST_TRESHOLD = 200    # if no kidnapping
    #ANGLE_TRESHOLD = 0.4
    
    DIST_TRESHOLD = 1000
    ANGLE_TRESHOLD = 6
    
    ClearView = False
    if(bool(sensor_package[0]) & bool(sensor_package[1]) & bool(sensor_package[2])):
        ClearView = True
    else:
        ClearView = False

    # 1) Time interval computation
    dt = time.time() - KF.get_time_stamp()
    KF.set_time_stamp(time.time())

    # 2) New Jacobian computation with new dt
    KF.recompute_F(dt)

    # 3) Prediction computation
    KF.predict()
    
    if(bool(sensor_package[0])):
        etat_predict = KF.current_estimate_state()
        
        angle_predict = np.squeeze(np.asarray(etat_predict[0]))
        angle_cam = sensor_package[2]
        diff_abs = angle_predict - angle_cam 
        diff_abs = np.sqrt(np.square(diff_abs))
        
        pos_predict=[etat_predict[0], etat_predict[1]]
        pos_predict=np.squeeze(np.asarray(pos_predict))

        sens_pos = [sensor_package[0], sensor_package[1]]

        dist=np.subtract(pos_predict,sens_pos)
        norme=np.sqrt(np.sum(np.square(dist)))

        if((norme>DIST_TRESHOLD) & (diff_abs>ANGLE_TRESHOLD)):
            print("Erratic measurement, only prediction")

        else:
            # 4) Prediction correction
            KF.update(sensor_package, ClearView)
    else:
        # 4) Prediction correction
        KF.update(sensor_package, ClearView)

    etat = KF.current_estimate_state()

    pos_angle = [etat[0], etat[1], etat[2]]
    


    return pos_angle