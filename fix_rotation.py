import numpy as np
from implementation.rotation import rot_X, rot_Y

def test_aero():
    alpha_0 = np.deg2rad(15)
    theta = np.deg2rad(60)
    F_inst = 10.0
    input_vector = np.array([0.0, 0.0, -F_inst])
    
    rotated_x = rot_X(theta) @ input_vector
    rotated_y = rot_Y(-alpha_0) @ rotated_x
    print(rotated_y)

test_aero()
