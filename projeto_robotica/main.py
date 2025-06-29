from drivers.simulation_driver.driver import SimulationDriver
import numpy as np

def main():
    driver = SimulationDriver()
   
    angles = [(np.radians(90), np.radians(90)), (np.radians(180), np.radians(0)), (np.radians(270), np.radians(270)), (np.radians(360), np.radians(0))]
    driver.move_angular_continuous(angles)

    driver = SimulationDriver()
    points = [(0, 2), (2, 0)]
    driver.move_eucl_continuous(points)
    
    print(driver.position)

if __name__ == "__main__":
    main()