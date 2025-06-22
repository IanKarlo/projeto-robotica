import numpy as np
import math
from functions.two_dim import fk, ik

def test_fk():
    test_cases = [
        # (theta1, theta2, expected_x, expected_y, expected_theta)
        (0, math.pi/2, 1 + 0, 0 + 1, math.pi/2),         # (1, 1)
        (math.pi/2, math.pi/2, 0 + -1, 1 + 0, math.pi),  # (-1, 1)
        (math.pi/2, -math.pi/2, 1, 1, 0),                # (1, 1)
        (-math.pi, math.pi, -1 + 1, 0 + 0, 0)            # (0, 0)
    ]

    for theta1, theta2, x_exp, y_exp, theta_exp in test_cases:
        x, y, theta = fk(theta1, theta2)
        assert np.isclose(x, x_exp, atol=1e-5), f"fk({theta1}, {theta2}): x={x} != {x_exp}"
        assert np.isclose(y, y_exp, atol=1e-5), f"fk({theta1}, {theta2}): y={y} != {y_exp}"
        assert np.isclose(theta, theta1 + theta2, atol=1e-5), f"fk({theta1}, {theta2}): theta={theta} != {theta_exp}"
        print(f"OK para ({theta1}, {theta2})")
    print("✅ test_fk passou")  

def test_ik():
    test_points = [
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
        (2, 1),
        (2, 0),
        (0, 2),
        (-2, 0)
    ]

    for x, y in test_points:
        try:
            (theta1_a, theta2_a), (theta1_b, theta2_b) = ik(x, y)

            # Valida A
            xy_a = fk(theta1_a, theta2_a)
            assert np.allclose(xy_a[:2], [x, y], atol=1e-6), f"Erro na solução A para ({x}, {y}): {xy_a}"

            # Valida B
            xy_b = fk(theta1_b, theta2_b)
            assert np.allclose(xy_b[:2], [x, y], atol=1e-6), f"Erro na solução B para ({x}, {y}): {xy_b}"

            print(f"OK para ({x}, {y})")

        except ValueError as e:
            print(f"Alvo inalcançável para ({x}, {y}): {e}")
            
    print("✅ test_ik passou")

if __name__ == "__main__":
    test_fk()
    test_ik()
    print("Todos os testes fk/ik foram executados ✅")