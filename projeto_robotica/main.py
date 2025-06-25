from drivers.simulation_driver.driver import SimulationDriver
import numpy as np

def main():
    driver = SimulationDriver()
    # points = [(2, 0), (0, 2), (-2, 0), (0, -2), (1, 0), (0, 1), (-1, 0), (0, -1), (2, 0)]
    # driver.move_eucl_continuous(points)
    # angles = [(np.radians(90), 0), (np.radians(180), 0), (np.radians(270), 0), (np.radians(360), 0)]
    # driver.move_angular_continuous(angles)
    # driver.move_cartesian(0, 2)
    points = [(0, 2), (-2, 0), (0, -2), (2, 0)]
    driver.move_eucl_continuous(points)
    print(driver.position)

if __name__ == "__main__":
    main()