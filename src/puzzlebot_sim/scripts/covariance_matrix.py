#!/usr/bin/env python
import numpy as np
np.set_printoptions(suppress=True)
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

delta_t = 0.1	# sampling time
v = 1	        # m/s
w = 1	        # rad/s

S_x = 0
S_y = 0
S_th = 0

m0 = np.array([[S_x],
            [S_y],
            [S_th]])	        # robot initial position

Sigma = np.array([[0,0,0],
                [0,0,0],
                [0,0,0]])	    # initial covariance matrix

q = np.array([[0.5, 0.01, 0.01],
            [0.01, 0.5, 0.01],
            [0.01, 0.01, 0.2]])	# noise

# Algoritmo
for i in range(2):
    m1 = np.array([[S_x + delta_t * v * np.cos(S_th)],
                [S_y + delta_t * v * np.sin(S_th)],
                [S_th + delta_t * w]])
    
    H = np.array([[1, 0, -delta_t * v * np.sin(S_th)],
                [0, 1, delta_t * v * np.cos(S_th)],
                [0, 0, 1]])

    S_x = m1[0]
    S_y = m1[1]
    S_th = m1[2]

    Sigma = np.array(H.dot(Sigma).dot(H.T)+q)
