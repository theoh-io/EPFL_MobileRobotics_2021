import time
from EKF_astolfi import ExtendedKalmanFilterAstolfi
import numpy as np

def kalman_step_complet(sensor_package, KF):
    DIST_TRESHOLD = 100
    
    ClearView = False
    # test si caméra thymio kidnappé, caméra cachée ou autre.
    # si un des elements du tableau est vide alors on retourne ClearView = false
    if(bool(sensor_package[0]) & bool(sensor_package[1]) & bool(sensor_package[2])):
        ClearView = True
    else:
        ClearView = False

    # 1) intervalle de temps entre derniere et nouveau predict/update
    dt = time.time() - KF.get_time_stamp()
    KF.set_time_stamp(time.time())

    # 2) recalcul de la transmission avec nouveau dt, et nouveau jacobien
    KF.recompute_F_and_Q(dt)
    
    # 3) Calcul de la prediction
    KF.predict()
    
    if(bool(sensor_package[0])):
        print("dans test de distance")
        etat_predict = KF.current_estimate_state()

        pos_predict=[etat_predict[0], etat_predict[1]]
        pos_predict=np.squeeze(np.asarray(pos_predict))

        sens_pos = [sensor_package[0], sensor_package[1]]

        dist=np.subtract(pos_predict,sens_pos)
        norme=np.sqrt(np.sum(np.square(dist)))

        if(norme>DIST_TRESHOLD):
            print("trop grande distance detectee, pas de update")
            print("position mesuree: ", sens_pos)
            print("position de la prediction: ", pos_predict)
            print("distance: ",norme)
        else:
            # 4) Correction de la prediction
            print("bonne distance detectee, update")
            KF.update(sensor_package, ClearView)
    else:
        KF.update(sensor_package, ClearView)

    etat = KF.current_estimate_state()

    pos_angle = [etat[0], etat[1], etat[2]]        #a verifier
    


    return pos_angle
   # return etat