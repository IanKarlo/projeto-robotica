from drivers.simulation_driver.driver import SimulationDriver
import numpy as np

def main():
    driver = SimulationDriver()
    # points = [(2, 0), (0, 2), (-2, 0), (0, -2), (1, 0), (0, 1), (-1, 0), (0, -1), (2, 0)]
    # driver.move_eucl_continuous(points)
    angles = [(np.radians(260), 0)]
    driver.move_angular_continuous(angles)
    print(driver.position)

if __name__ == "__main__":
    main()