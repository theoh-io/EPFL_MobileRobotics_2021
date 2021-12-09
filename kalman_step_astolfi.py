import time
from EKF_complet import ExtendedKalmanFilterComplete

def kalman_step_complet(sensor_package):

    # 1) intervalle de temps entre derniere et nouveau predict/update
    dt = time.time() - KF.get_time_stamp()
    KF.set_time_stamp(time.time())

    # 2) recalcul de la transmission avec nouveau dt, et nouveau jacobien
    KF.recompute_F_and_Q(dt)

    # 3) Calcul de la prediction
    KF.predict()

    # 4) Correction de la prediction
    KF.update(rotation, sensor_package)

    return KF.current_estimate_state()