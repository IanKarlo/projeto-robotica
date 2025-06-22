from typing import List, Tuple
import numpy as np
from functions.trajectory.motor import MotorLimits
from functions.two_dim import ik
import matplotlib.pyplot as plt
import os

class TrajectoryPoint:
    """Ponto da trajetória"""
    time: float
    position: float
    velocity: float
    acceleration: float

    def __init__(self, time: float, position: float, velocity: float, acceleration: float):
        self.time = time
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration

class TrapezoidalTrajectoryPlanner:
    def __init__(self, motor1_limits: MotorLimits, motor2_limits: MotorLimits):
        self.motor1_limits = motor1_limits
        self.motor2_limits = motor2_limits
    
    def calculate_single_motor_trajectory(self, distance: float, limits: MotorLimits) -> Tuple[float, float, float, float]:
        """
        Calcula os tempos do perfil trapezoidal para um motor
        Retorna: (t_accel, t_cruise, t_decel, total_time)
        """
        v_max = limits.max_velocity
        a_max = limits.max_acceleration
        
        # Tempo para acelerar até velocidade máxima
        t_accel = v_max / a_max
        
        # Distância percorrida durante aceleração e desaceleração
        d_accel = 0.5 * a_max * t_accel**2
        d_decel = d_accel  # assumindo desaceleração simétrica
        
        # Verifica se é possível atingir velocidade máxima
        if (d_accel + d_decel) >= abs(distance):
            # Perfil triangular - não atinge velocidade máxima
            t_accel = np.sqrt(abs(distance) / a_max)
            t_cruise = 0
            t_decel = t_accel
            v_cruise = a_max * t_accel
        else:
            # Perfil trapezoidal - atinge velocidade máxima
            d_cruise = abs(distance) - d_accel - d_decel
            t_cruise = d_cruise / v_max
            t_decel = t_accel
            v_cruise = v_max
        
        total_time = t_accel + t_cruise + t_decel
        return t_accel, t_cruise, t_decel, total_time, v_cruise
    
    def plan_synchronized_trajectory(self, distance1: float, distance2: float, 
                                   time_step: float = 0.01) -> Tuple[List[TrajectoryPoint], List[TrajectoryPoint]]:
        """
        Planeja trajetórias sincronizadas para ambos os motores
        """
        # Calcula trajetórias individuais
        t1_accel, t1_cruise, t1_decel, total_time1, v1_cruise = self.calculate_single_motor_trajectory(
            distance1, self.motor1_limits)
        t2_accel, t2_cruise, t2_decel, total_time2, v2_cruise = self.calculate_single_motor_trajectory(
            distance2, self.motor2_limits)
        
        # Usa o tempo maior para sincronizar
        sync_time = max(total_time1, total_time2)
        
        # Recalcula os perfis para o tempo sincronizado
        traj1 = self._calculate_synchronized_profile(distance1, sync_time, self.motor1_limits, time_step)
        traj2 = self._calculate_synchronized_profile(distance2, sync_time, self.motor2_limits, time_step)

        debug = os.getenv("DEBUG", "False") == "True"

        if debug:
            print(f"Tempo total sincronizado: {sync_time:.3f}s")
            print(f"Motor 1 - Distância: {distance1}, Tempo original: {total_time1:.3f}s")
            print(f"Motor 2 - Distância: {distance2}, Tempo original: {total_time2:.3f}s")
            plot_trajectories(traj1, traj2)
        
        # print(f"Tempo total sincronizado: {sync_time:.3f}s")
        # print(f"Motor 1 - Distância: {distance1}, Tempo original: {total_time1:.3f}s")
        # print(f"Motor 2 - Distância: {distance2}, Tempo original: {total_time2:.3f}s")
        
        return traj1, traj2
    
    def _calculate_synchronized_profile(self, distance: float, target_time: float, 
                                      limits: MotorLimits, time_step: float) -> List[TrajectoryPoint]:
        """
        Calcula o perfil de movimento para atingir a distância no tempo alvo
        """
        # Estratégia: ajustar a velocidade máxima para caber no tempo disponível
        a_max = limits.max_acceleration
        v_max_available = limits.max_velocity
        
        # Calcula velocidade necessária para completar no tempo alvo
        # Usando equação: d = v_max * (t - v_max/a) onde t >= 2*v_max/a
        # Resolvendo para v_max: v_max = (a*t ± sqrt((a*t)² - 4*a*d)) / 2
        
        discriminant = (a_max * target_time)**2 - 4 * a_max * abs(distance)
        
        if discriminant >= 0:
            v_max_needed = (a_max * target_time - np.sqrt(discriminant)) / 2
            v_max_used = min(v_max_needed, v_max_available)
        else:
            # Não é possível completar no tempo com a aceleração disponível
            v_max_used = v_max_available
        
        # Recalcula os tempos com a velocidade ajustada
        t_accel = v_max_used / a_max
        t_decel = t_accel
        
        if (t_accel + t_decel) >= target_time:
            # Perfil triangular
            t_accel = target_time / 2
            t_cruise = 0
            t_decel = target_time / 2
            v_max_used = a_max * t_accel
        else:
            # Perfil trapezoidal
            t_cruise = target_time - t_accel - t_decel
        
        # Gera pontos da trajetória
        trajectory = []
        time_points = np.arange(0, target_time + time_step, time_step)
        
        direction = 1 if distance >= 0 else -1
        
        for t in time_points:
            if t <= t_accel:
                # Fase de aceleração
                pos = direction * 0.5 * a_max * t**2
                vel = direction * a_max * t
                acc = direction * a_max
            elif t <= (t_accel + t_cruise):
                # Fase de velocidade constante
                pos = direction * (0.5 * a_max * t_accel**2 + v_max_used * (t - t_accel))
                vel = direction * v_max_used
                acc = 0
            else:
                # Fase de desaceleração
                t_dec = t - t_accel - t_cruise
                pos = direction * (0.5 * a_max * t_accel**2 + v_max_used * t_cruise + 
                                 v_max_used * t_dec - 0.5 * a_max * t_dec**2)
                vel = direction * (v_max_used - a_max * t_dec)
                acc = direction * (-a_max)
            
            trajectory.append(TrajectoryPoint(t, pos, vel, acc))
        
        return trajectory
    
def traj_joint(theta1_start: float, theta2_start: float, theta1_end: float, theta2_end: float, planner: TrapezoidalTrajectoryPlanner, time_step: float = 0.01):
    distance, distance2 = calculate_distances(theta1_start, theta1_end, theta2_start, theta2_end)
    trajectory1, trajectory2 = planner.plan_synchronized_trajectory(distance, distance2, time_step)
    q_values = []
    for i in range(len(trajectory1)):
        q_values.append([trajectory1[i].position, trajectory2[i].position])
    return q_values

def traj_eucl(theta1_start: float, theta2_start: float, x_end: float, y_end: float, planner: TrapezoidalTrajectoryPlanner, time_step: float = 0.01):
    target_angles1, target_angles2 = ik(x_end, y_end)
    trajectories1 = traj_joint(theta1_start, theta2_start, target_angles1[0], target_angles1[1], planner, time_step)
    trajectories2 = traj_joint(theta1_start, theta2_start, target_angles2[0], target_angles2[1], planner, time_step)

    if len(trajectories1) < len(trajectories2):
        return trajectories1, target_angles1
    else:
        return trajectories2, target_angles2


def calculate_distances(theta1_start: float, theta1_end: float, theta2_start: float, theta2_end: float):
    """Calcula a distância mínima entre ângulos (menor caminho angular)"""
    # Normaliza a diferença para o intervalo [-pi, pi]
    distance1 = ((theta1_end - theta1_start + np.pi) % (2 * np.pi)) - np.pi
    distance2 = ((theta2_end - theta2_start + np.pi) % (2 * np.pi)) - np.pi
    return distance1, distance2

def plot_trajectories(traj1: List[TrajectoryPoint], traj2: List[TrajectoryPoint]):
    """Plota as trajetórias dos dois motores"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    times1 = [p.time for p in traj1]
    positions1 = [p.position for p in traj1]
    velocities1 = [p.velocity for p in traj1]
    accelerations1 = [p.acceleration for p in traj1]
    
    times2 = [p.time for p in traj2]
    positions2 = [p.position for p in traj2]
    velocities2 = [p.velocity for p in traj2]
    accelerations2 = [p.acceleration for p in traj2]
    
    # Posição
    axes[0].plot(times1, positions1, 'b-', label='Motor 1', linewidth=2)
    axes[0].plot(times2, positions2, 'r--', label='Motor 2', linewidth=2)
    axes[0].set_ylabel('Posição')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Trajetórias Sincronizadas')
    
    # Velocidade
    axes[1].plot(times1, velocities1, 'b-', label='Motor 1', linewidth=2)
    axes[1].plot(times2, velocities2, 'r--', label='Motor 2', linewidth=2)
    axes[1].set_ylabel('Velocidade')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Aceleração
    axes[2].plot(times1, accelerations1, 'b-', label='Motor 1', linewidth=2)
    axes[2].plot(times2, accelerations2, 'r--', label='Motor 2', linewidth=2)
    axes[2].set_ylabel('Aceleração')
    axes[2].set_xlabel('Tempo (s)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()