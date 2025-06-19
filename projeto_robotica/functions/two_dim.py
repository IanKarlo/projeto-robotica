import numpy as np
from .one_dim import SE2_theta, SE2_xy

L1 = 1
L2 = 1

def fk(theta1, theta2):
    
    T1 = SE2_theta(theta1) @ SE2_xy(L1, 0)

    # print(T1)

    T2 = SE2_theta(theta2) @ SE2_xy(L2, 0)

    # print(T2)

    T_total = T1 @ T2

    pos = np.asarray(T_total @ np.array([0, 0, 1])).flatten()
    x, y = pos[0], pos[1]

    orientation = theta1 + theta2

    return x, y, orientation

def ik(x, y):
    # Compute cos(theta2)
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)

    # Checa se está no domínio válido
    if np.abs(D) > 1:
        raise ValueError("Alvo inalcançável: |D| > 1")

    # Duas soluções possíveis para theta2
    theta2_a = np.arctan2( +np.sqrt(1 - D**2), D )  # cotovelo para cima
    theta2_b = np.arctan2( -np.sqrt(1 - D**2), D )  # cotovelo para baixo

    # Para cada theta2, calcula theta1 correspondente
    def theta1(theta2):
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        return np.arctan2(y, x) - np.arctan2(k2, k1)

    theta1_a = theta1(theta2_a)
    theta1_b = theta1(theta2_b)

    # Retorna as duas soluções
    return (theta1_a, theta2_a), (theta1_b, theta2_b)