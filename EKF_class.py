import numpy as np
import numpy.matlib
from math import pi, sqrt


class ExtendedKalmanFilterAstolfi:
    def __init__(self):
        self.__xI = np.matlib.identity(8)
        self.__x = None
        self.__F = None
        self.__Q = None

        self.__P = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1]])


        self.__Hcamspeed = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0, 0]])
        
        self.__Hspeed = np.matrix([[0, 0, 0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, 0, 0, 0]])

        self.__Rcamspeed = np.matrix([[2.5, 0,   0, 0, 0],
                              [0, 2.5,   0, 0, 0],
                              [0, 0, 0.005, 0, 0],
                              [0, 0,   0, 6, 0],
                              [0, 0,   0, 0, 6]])
        
        self.__Rspeed = np.matrix([[6, 0],
                              [0, 6]])

        self.__timeStamp = None

    @property
    def current_estimate(self):
        return self.__x, self.__P

    def current_estimate_state(self):
        return self.__x

    def init_state_vector(self, posx, posy, angle, vit_roue_droite, vit_roue_gauche, vx, vy, vitesse_rota):
        self.__x = np.matrix([[posx, posy, angle, vit_roue_droite, vit_roue_gauche,  vx, vy, vitesse_rota]]).T

    def set_time_stamp(self, dt):
        self.__timeStamp = dt

    def get_time_stamp(self):
        return self.__timeStamp

    def recompute_F(self, dt):
        '''
        updates the motion model and process covar based on delta time from last measurement.
        '''

        # set F
        etat = self.current_estimate

        L_ROUE_CENTRE = 45 # in mm
        alpha = etat[0][2].item(0)
        NroueDroite = etat[0][3].item(0)
        NroueGauche = etat[0][4].item(0)

        alpha_sin = np.sin(alpha)
        alpha_cos = np.cos(alpha)

        e52 = -alpha_sin*(NroueDroite + NroueGauche)/2
        e53 = alpha_cos/2
        e54 = alpha_cos/2

        e62 = -alpha_cos*(NroueDroite+NroueGauche)/2
        e63 = -alpha_sin/2
        e64 = -alpha_sin/2

        e73 = 1/(2*L_ROUE_CENTRE)
        e74 = -1/(2*L_ROUE_CENTRE)

        e05 = e16 = e27 = dt

        self.__F = np.matrix([[1, 0,   0,   0, 0,   e05,   0,   0],
                              [0, 1,   0,   0, 0,     0, e16,   0],
                              [0, 0,   1,   0, 0,     0,   0, e27],
                              [0, 0,   0,   1,   0,   0,   0,   0],
                              [0, 0,   0,   0,   1,   0,   0,   0],
                              [0, 0, e52, e53, e54,   0,   0,   0],
                              [0, 0, e62, e63, e64,   0,   0,   0],
                              [0, 0,   0, e73, e74,   0,   0,   0]])
        # set Q
        e00 = 2.5
        e11 = 2.5
        e22 = 0.05
        e33 = 6
        e44 = 6
        e55 = 1
        e66 = 1
        e77 = 1


        self.__Q = np.matrix([[e00, 0, 0, 0, 0, 0, 0, 0],
                              [0, e11, 0, 0, 0, 0, 0, 0],
                              [0, 0, e22, 0, 0, 0, 0, 0],
                              [0, 0, 0, e33, 0, 0, 0, 0],
                              [0, 0, 0, 0, e44, 0, 0, 0],
                              [0, 0, 0, 0, 0, e55, 0, 0],
                              [0, 0, 0, 0, 0, 0, e66, 0],
                              [0, 0, 0, 0, 0, 0, 0, e77]])

    def predict(self):
        '''
        This is a projection step. we predict into the future.
        '''

        self.__x = self.__F * self.__x
        self.__P = (self.__F * self.__P * self.__F.T) + self.__Q

    def update(self, sensor_package, ClearView):
        '''
        This is the projection correction; after we predict we use the sensor data
        and use the kalman gain to figure out how much of the correction we need.
        '''
        SPEED_CORRECTION = 0.6
        SPEED_CONV_FACT = 0.38

        posx = sensor_package[0]
        posy = sensor_package[1]
        angle_sensor = sensor_package[2]

        vit_roue_droite = sensor_package[3]*SPEED_CONV_FACT*SPEED_CORRECTION
        vit_roue_gauche = sensor_package[4]*SPEED_CONV_FACT*SPEED_CORRECTION

        if(ClearView):
            # this is the error of our prediction to the sensor readings
            y = [[posx], [posy], [angle_sensor], [vit_roue_droite], [vit_roue_gauche]] - self.__Hcamspeed * self.__x

            # pre compute for the kalman gain K
            PHLt = self.__P * self.__Hcamspeed.T
            S = self.__Hcamspeed * PHLt + self.__Rcamspeed
            K = PHLt * S.I
            # now we update our prediction using the error and kalman gain.
            self.__x += K * y
            self.__P = (self.__xI - K * self.__Hcamspeed) * self.__P

        else:
            # this is the error of our prediction to the sensor readings
            y = [[vit_roue_droite], [vit_roue_gauche]] - self.__Hspeed * self.__x

            # pre compute for the kalman gain K
            PHLt = self.__P * self.__Hspeed.T
            S = self.__Hspeed * PHLt + self.__Rspeed
            K = PHLt * S.I
             # now we update our prediction using the error and kalman gain.
            self.__x += K * y
            self.__P = (self.__xI - K * self.__Hspeed) * self.__P

       