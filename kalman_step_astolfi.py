import time
from EKF_astolfi import ExtendedKalmanFilterAstolfi

def kalman_step_complet(sensor_package, KF):
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

    # 4) Correction de la prediction
    KF.update(sensor_package, ClearView)

    etat = KF.current_estimate_state()

    pos_angle = [etat[0], etat[1], etat[2]]        #a verifier

    return pos_angle