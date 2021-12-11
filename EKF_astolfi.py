import numpy as np
import numpy.matlib
from math import pi, sqrt


class ExtendedKalmanFilterAstolfi:
    def __init__(self):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        TODO: consider making these just class methods; that way we don't have many instances and instead
        each tracker will just call to these methods with the matrices to update.
        '''

        # do this once do we don't keep redoing in update step
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


        self.__H = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0, 0]])  # matrice si thymio repéré
        
        self.__Hkidnap = np.matrix([[0, 0, 0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, 0, 0, 0]])  # matrice si thymio non repéré

        # à calibrer
        self.__R = np.matrix([[1, 0,   0, 0, 0],
                              [0, 1,   0, 0, 0],
                              [0, 0, 0.1, 0, 0],
                              [0, 0,   0, 1, 0],
                              [0, 0,   0, 0, 1]])
        
        self.__Rkidnap = np.matrix([[1, 0],
                              [0, 1]])

        # we can adjust these to get better accuracy
        self.__noise_ax = 1
        self.__noise_ay = 1

        # timestamp du dernier sample
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

    def recompute_F_and_Q(self, dt):            # xxx a modifier : Q ne se fait pas modifier, que F
        '''
        updates the motion model and process covar based on delta time from last measurement.
        '''

        # set F
        etat = self.current_estimate

        L_ROUE_CENTRE = 45 # en mm, car vitesse de roue est en mm/s
        alpha = etat[0][2].item(0)  # angle de l'etat
        NroueDroite = etat[0][3].item(0)  # tirer la vitesse
        NroueGauche = etat[0][4].item(0)  # tirer la vitesse
        print("Vitesses droite / gauche -------------------")
        print(NroueDroite)
        print(NroueGauche)
        print("--------------------------------------------")
        #thymio tourne a gauche : rotation positive car anti horaire

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
        print("temps entre samples: ", dt)

        self.__F = np.matrix([[1, 0,   0,   0, 0,   e05,   0,   0],
                              [0, 1,   0,   0, 0,     0, e16,   0],
                              [0, 0,   1,   0, 0,     0,   0, e27],
                              [0, 0,   0,   1,   0,   0,   0,   0],
                              [0, 0,   0,   0,   1,   0,   0,   0],
                              [0, 0, e52, e53, e54,   0,   0,   0],
                              [0, 0, e62, e63, e64,   0,   0,   0],
                              [0, 0,   0, e73, e74,   0,   0,   0]])
        # set Q
        e00 = 1
        e11 = 1
        e22 = 1
        e33 = 1
        e44 = 1
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

        SPEED_CONV_FACT = 0.38

        posx = sensor_package[0]
        posy = sensor_package[1]
        angle_sensor = sensor_package[2]
        vit_roue_droite = sensor_package[3]*SPEED_CONV_FACT
        vit_roue_gauche = sensor_package[4]*SPEED_CONV_FACT
        
        #clearview = thymio reperé 

        if(ClearView):
            # this is the error of our prediction to the sensor readings
            y = [[posx], [posy], [angle_sensor], [vit_roue_droite], [vit_roue_gauche]] - self.__H * self.__x

            # pre compute for the kalman gain K
            PHLt = self.__P * self.__H.T
            S = self.__H * PHLt + self.__R
            K = PHLt * S.I
            # now we update our prediction using the error and kalman gain.
            self.__x += K * y
            self.__P = (self.__xI - K * self.__H) * self.__P
            
        else:
            # this is the error of our prediction to the sensor readings
            y = [[vit_roue_droite], [vit_roue_gauche]] - self.__Hkidnap * self.__x

            # pre compute for the kalman gain K
            PHLt = self.__P * self.__Hkidnap.T
            S = self.__Hkidnap * PHLt + self.__Rkidnap
            K = PHLt * S.I
             # now we update our prediction using the error and kalman gain.
            self.__x += K * y
            self.__P = (self.__xI - K * self.__Hkidnap) * self.__P

       