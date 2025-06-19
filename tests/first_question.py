import numpy as np
import math
from functions.one_dim import SE2_xy, SE2_theta

P = np.array([0.5, 0.5, 1]).reshape(3, 1)

def test_1_p_em_R1():
    P_R1 = P[:2].flatten()
    assert np.allclose(P_R1, [0.5, 0.5]), f"Esperado [0.5, 0.5], obtido {P_R1}"

def test_2_p_em_R2():
    T = SE2_xy(1, 0.25)
    P_R2 = np.linalg.inv(T) @ P
    P_R2 = P_R2[:2].flatten()
    assert np.allclose(P_R2, [-0.5, 0.25]), f"Esperado [-0.5, 0.5], obtido {P_R2}"

def test_3_p_em_R1_com_rotacao():
    T = SE2_xy(1, 0.25) @ SE2_theta(math.radians(45))
    P_R2 = np.linalg.inv(T) @ P
    P_R1 = T @ P_R2
    assert np.allclose(P_R1[:2].flatten(), [0.5, 0.5], atol=1e-6)

def test_4_p_em_R2_com_rotacao():
    T = SE2_xy(1, 0.25) @ SE2_theta(math.radians(45))
    P_R2 = np.linalg.inv(T) @ P
    esperado = np.array([- 0.17677, 0.53033])
    assert np.allclose(P_R2[:2].flatten(), esperado, atol=1e-5), f"Esperado {esperado}, obtido {P_R2[:2].flatten()}"

if __name__ == "__main__":
    test_1_p_em_R1()
    test_2_p_em_R2()
    test_3_p_em_R1_com_rotacao()
    test_4_p_em_R2_com_rotacao()
    print("Todos os testes passaram!")