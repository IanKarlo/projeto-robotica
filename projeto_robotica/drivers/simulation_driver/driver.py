import numpy as np
from functions.two_dim import fk
from functions.trajectory.trajectory import TrapezoidalTrajectoryPlanner, traj_joint, traj_eucl
from functions.trajectory.motor import MotorLimits
from roboticstoolbox import DHRobot, RevoluteDH
from typing import List, Tuple

class SimulationPosition:
    def __init__(self, x: float = 2, y: float = 0, theta1: float = 0, theta2: float = 0):
        self.x = x # terminal X position
        self.y = y # terminal Y position
        self.theta1 = theta1 # joint 1 angle
        self.theta2 = theta2 # joint 2 angle

    def __str__(self):
        return f"Position(x={self.x}, y={self.y}, theta1={self.theta1}, theta2={self.theta2})"

    def update_position(self, x: float, y: float, theta1: float, theta2: float):
        self.x = x
        self.y = y
        self.theta1 = theta1
        self.theta2 = theta2

class SimulationDriver:
    """Driver da simulação"""
    def __init__(self, max_velocity: float = np.pi, max_acceleration: float = np.pi*10):
        motor_limits1 = MotorLimits(max_velocity, max_acceleration)
        motor_limits2 = MotorLimits(max_velocity, max_acceleration)
        self.planner = TrapezoidalTrajectoryPlanner(motor_limits1, motor_limits2)
        self.position = SimulationPosition()

    def move_angular(self, theta1: float, theta2: float, time_step: float = 0.01):
        trajectory = traj_joint(self.position.theta1, self.position.theta2, theta1, theta2, self.planner, time_step)
        theta1_start, theta2_start = self.position.theta1, self.position.theta2
        self.update_postion(theta1, theta2)
        new_trajectory = [[x+theta1_start, y+theta2_start] for x, y in trajectory]
        self.__robot_simulation(new_trajectory)

    def move_eucl(self, x: float, y: float, time_step: float = 0.01):
        trajectory, angles = traj_eucl(self.position.theta1, self.position.theta2, x, y, self.planner, time_step)
        theta1_start, theta2_start = self.position.theta1, self.position.theta2
        self.update_postion(angles[0], angles[1])
        new_trajectory = [[x+theta1_start, y+theta2_start] for x, y in trajectory]
        self.__robot_simulation(new_trajectory)

    def move_eucl_continuous(self, points: List[Tuple[float, float]], time_step: float = 0.01):
        theta1_start, theta2_start = self.position.theta1, self.position.theta2
        final_trajectory = []
        for point in points:
            trajectory, angles = traj_eucl(theta1_start, theta2_start, point[0], point[1], self.planner, time_step)
            new_trajectory = [[x+theta1_start, y+theta2_start] for x, y in trajectory]
            final_trajectory = [*final_trajectory, *new_trajectory]
            theta1_start, theta2_start = angles[0], angles[1]
        self.update_postion(theta1_start, theta2_start)
        self.__robot_simulation(final_trajectory)
    
    def move_angular_continuous(self, points: List[Tuple[float, float]], time_step: float = 0.01):
        theta1_start, theta2_start = self.position.theta1, self.position.theta2
        final_trajectory = []
        for point in points:
            trajectory = traj_joint(theta1_start, theta2_start, point[0], point[1], self.planner, time_step)
            new_trajectory = [[x+theta1_start, y+theta2_start] for x, y in trajectory]
            final_trajectory = [*final_trajectory, *new_trajectory]
            theta1_start, theta2_start = point[0], point[1]
        self.update_postion(theta1_start, theta2_start)
        self.__robot_simulation(final_trajectory)
        
    
    def update_postion(self, theta1: float, theta2: float):
        x, y, _ = fk(theta1, theta2)
        self.position.update_position(x, y, theta1, theta2)


    def __robot_simulation(self, trajectory, time_step: float = 0.01):
        # Criar um robô planar 2 DOF
        link1 = RevoluteDH(a=1.0, alpha=0)
        link2 = RevoluteDH(a=1.0, alpha=0)
        robot = DHRobot([link1, link2], name="Planar2DOF")
        
        q_array = np.array(trajectory)
        
        file_name = f"outputs/robot_trajectory_{time_step}.gif"

        # Plotar a trajetória do robô
        robot.plot(
            q_array,
            dt=time_step,
            backend="pyplot",
            movie=file_name,
            block=False
        )