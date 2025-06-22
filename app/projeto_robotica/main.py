from drivers.simulation_driver.driver import SimulationDriver
import numpy as np

driver = SimulationDriver()
# driver.move_angular(np.pi*3/2, np.pi/2)
# driver.move_angular(np.pi/8, np.pi/2)
# driver.move_eucl(0.5, 0.5)
# driver.move_eucl(2, 0)
# driver.move_eucl(0.5, 0.5)
# points = [(0.5, 0.5), (2, 0), (0.5, 0.5)]
# driver.move_eucl_continuous(points)
angles = [(np.pi/8, 0), (np.pi/2, 0)]
driver.move_angular_continuous(angles)
print(driver.position)