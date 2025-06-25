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

def traj_cart(theta1_start: float, theta2_start: float, x_end: float, y_end: float, planner: TrapezoidalTrajectoryPlanner, time_step: float = 0.01):
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


def traj_eucl(x_start: float, y_start: float, x_end: float, y_end: float, 
                  theta1_start: float, theta2_start: float, 
                  planner: TrapezoidalTrajectoryPlanner, time_step: float = 0.01,
                  max_velocity: float = 0.5, max_acceleration: float = 0.5):
    
    cartesian_distance = np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)
    

    dummy_limits = MotorLimits(max_velocity=max_velocity, max_acceleration=max_acceleration)
    t_accel, t_cruise, t_decel, total_time, v_cruise = planner.calculate_single_motor_trajectory(
        cartesian_distance, dummy_limits)
    
    debug = os.getenv("DEBUG", "False") == "True"
    if debug:
        print(f"Cartesian trajectory from ({x_start}, {y_start}) to ({x_end}, {y_end})")
        print(f"Cartesian distance: {cartesian_distance:.3f}")
        print(f"Total time: {total_time:.3f}s")
        print(f"Acceleration time: {t_accel:.3f}s, Cruise time: {t_cruise:.3f}s, Deceleration time: {t_decel:.3f}s")
        print(f"Cruise velocity: {v_cruise:.3f} units/s")
        print(f"Maximum acceleration: {dummy_limits.max_acceleration:.3f} units/s²")
    
    time_points = np.arange(0, total_time + time_step, time_step)
    
    trajectory_points = []
    cartesian_points = []
    cartesian_velocities = []
    
    dx = x_end - x_start
    dy = y_end - y_start
    
    magnitude = np.sqrt(dx**2 + dy**2)
    if magnitude > 0:
        unit_dx = dx / magnitude
        unit_dy = dy / magnitude
    else:
        unit_dx = 0
        unit_dy = 0
    
    current_theta1 = theta1_start
    current_theta2 = theta2_start
    
    for t in time_points:
        if t <= t_accel:
            distance = 0.5 * dummy_limits.max_acceleration * t**2
            velocity = dummy_limits.max_acceleration * t
            acceleration = dummy_limits.max_acceleration
        elif t <= (t_accel + t_cruise):
            distance = 0.5 * dummy_limits.max_acceleration * t_accel**2 + v_cruise * (t - t_accel)
            velocity = v_cruise
            acceleration = 0
        else:
            t_dec = t - t_accel - t_cruise
            distance = (0.5 * dummy_limits.max_acceleration * t_accel**2 + 
                       v_cruise * t_cruise + 
                       v_cruise * t_dec - 0.5 * dummy_limits.max_acceleration * t_dec**2)
            velocity = v_cruise - dummy_limits.max_acceleration * t_dec
            acceleration = -dummy_limits.max_acceleration
        
        ratio = distance / cartesian_distance if cartesian_distance > 0 else 0
        x = x_start + ratio * dx
        y = y_start + ratio * dy
        
        cartesian_points.append((x, y))
        cartesian_velocities.append((velocity * unit_dx, velocity * unit_dy))
        
        try:
            angles1, angles2 = ik(x, y)
            
            dist1 = sum(abs(np.array(angles1) - np.array([current_theta1, current_theta2])))
            dist2 = sum(abs(np.array(angles2) - np.array([current_theta1, current_theta2])))
            
            if dist1 <= dist2:
                trajectory_points.append(list(angles1))
                chosen_angles = angles1
            else:
                trajectory_points.append(list(angles2))
                chosen_angles = angles2
                
            current_theta1, current_theta2 = chosen_angles
            
        except ValueError as e:
            print(f"Warning: Point ({x}, {y}) is unreachable: {e}")
            continue
    
    if debug and trajectory_points:
        theta1_traj = [point[0] for point in trajectory_points]
        theta2_traj = [point[1] for point in trajectory_points]
        
        traj1 = []
        traj2 = []
        for i, t in enumerate(time_points[:len(trajectory_points)]):
            traj1.append(TrajectoryPoint(t, theta1_traj[i], 0.0, 0.0))
            traj2.append(TrajectoryPoint(t, theta2_traj[i], 0.0, 0.0))
        
        filename_prefix = f"cartesian_{x_start:.1f}_{y_start:.1f}_to_{x_end:.1f}_{y_end:.1f}"
        
        times = time_points[:len(cartesian_points)]
        x_positions = [p[0] for p in cartesian_points]
        y_positions = [p[1] for p in cartesian_points]
        x_velocities = [v[0] for v in cartesian_velocities]
        y_velocities = [v[1] for v in cartesian_velocities]
        
        x_accelerations = []
        y_accelerations = []
        for i in range(len(times)):
            if i == 0:
                x_acc = 0
                y_acc = 0
            else:
                dt = times[i] - times[i-1]
                if dt > 0:
                    x_acc = (x_velocities[i] - x_velocities[i-1]) / dt
                    y_acc = (y_velocities[i] - y_velocities[i-1]) / dt
                else:
                    x_acc = 0
                    y_acc = 0
            x_accelerations.append(x_acc)
            y_accelerations.append(y_acc)
        
        
        plot_cartesian_trajectory(trajectory_points, x_start, y_start)
        
        plot_cartesian_trajectories(times, x_positions, y_positions, 
                                x_velocities, y_velocities, 
                                x_accelerations, y_accelerations)
        
        save_trajectory_plots(traj1, traj2, filename_prefix)
        save_cartesian_trajectory(trajectory_points, x_start, y_start, filename_prefix)
        save_cartesian_trajectories(times, x_positions, y_positions, 
                                x_velocities, y_velocities, 
                                x_accelerations, y_accelerations,
                                filename_prefix)
    
    # Return the trajectory and the final angles
    return trajectory_points, (current_theta1, current_theta2)

def plot_cartesian_trajectory(trajectory_points, x_start, y_start):
    """
    Plot the Cartesian trajectory
    """
    from functions.two_dim import fk
    
    # Extract x, y coordinates from joint angles
    x_coords = []
    y_coords = []
    
    for point in trajectory_points:
        x, y, _ = fk(point[0], point[1])
        x_coords.append(x)
        y_coords.append(y)
    
    plt.figure(figsize=(8, 8))
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)
    plt.plot(x_start, y_start, 'go', markersize=10)  # Start point
    plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10)  # End point
    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Cartesian Trajectory')
    plt.show()

def save_trajectory_plots(traj1: List[TrajectoryPoint], traj2: List[TrajectoryPoint], filename_prefix: str = "trajectory"):
    """Saves the trajectory plots to files instead of displaying them"""
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
    
    # Ensure the outputs directory exists
    os.makedirs("outputs", exist_ok=True)
    
    # Save the figure
    plt.savefig(f"outputs/{filename_prefix}_joint_space.png")
    plt.close(fig)

def save_cartesian_trajectory(trajectory_points, x_start, y_start, filename_prefix: str = "trajectory"):
    """
    Save the Cartesian trajectory plot to a file
    """
    from functions.two_dim import fk
    
    # Extract x, y coordinates from joint angles
    x_coords = []
    y_coords = []
    
    for point in trajectory_points:
        x, y, _ = fk(point[0], point[1])
        x_coords.append(x)
        y_coords.append(y)
    
    plt.figure(figsize=(8, 8))
    plt.plot(x_coords, y_coords, 'b-', linewidth=2)
    plt.plot(x_start, y_start, 'go', markersize=10)  # Start point
    plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10)  # End point
    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Cartesian Trajectory')
    
    # Ensure the outputs directory exists
    os.makedirs("outputs", exist_ok=True)
    
    # Save the figure
    plt.savefig(f"outputs/{filename_prefix}_cartesian_space.png")
    plt.close()

def plot_cartesian_trajectories(times: List[float], 
                               x_positions: List[float], y_positions: List[float],
                               x_velocities: List[float], y_velocities: List[float],
                               x_accelerations: List[float], y_accelerations: List[float]):
    """
    Plot the trajectories of X and Y coordinates in Cartesian space.
    
    Args:
        times: List of time points
        x_positions: List of X positions
        y_positions: List of Y positions
        x_velocities: List of X velocities
        y_velocities: List of Y velocities
        x_accelerations: List of X accelerations
        y_accelerations: List of Y accelerations
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Position
    axes[0].plot(times, x_positions, 'b-', label='X Position', linewidth=2)
    axes[0].plot(times, y_positions, 'r--', label='Y Position', linewidth=2)
    axes[0].set_ylabel('Position')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Cartesian Trajectories')
    
    # Velocity
    axes[1].plot(times, x_velocities, 'b-', label='X Velocity', linewidth=2)
    axes[1].plot(times, y_velocities, 'r--', label='Y Velocity', linewidth=2)
    axes[1].set_ylabel('Velocity')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Acceleration
    axes[2].plot(times, x_accelerations, 'b-', label='X Acceleration', linewidth=2)
    axes[2].plot(times, y_accelerations, 'r--', label='Y Acceleration', linewidth=2)
    axes[2].set_ylabel('Acceleration')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def save_cartesian_trajectories(times: List[float], 
                               x_positions: List[float], y_positions: List[float],
                               x_velocities: List[float], y_velocities: List[float],
                               x_accelerations: List[float], y_accelerations: List[float],
                               filename_prefix: str = "cartesian_trajectory"):
    """
    Save the trajectories of X and Y coordinates in Cartesian space to a file.
    
    Args:
        times: List of time points
        x_positions: List of X positions
        y_positions: List of Y positions
        x_velocities: List of X velocities
        y_velocities: List of Y velocities
        x_accelerations: List of X accelerations
        y_accelerations: List of Y accelerations
        filename_prefix: Prefix for the output filename
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Position
    axes[0].plot(times, x_positions, 'b-', label='X Position', linewidth=2)
    axes[0].plot(times, y_positions, 'r--', label='Y Position', linewidth=2)
    axes[0].set_ylabel('Position')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Cartesian Trajectories')
    
    # Velocity
    axes[1].plot(times, x_velocities, 'b-', label='X Velocity', linewidth=2)
    axes[1].plot(times, y_velocities, 'r--', label='Y Velocity', linewidth=2)
    axes[1].set_ylabel('Velocity')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Acceleration
    axes[2].plot(times, x_accelerations, 'b-', label='X Acceleration', linewidth=2)
    axes[2].plot(times, y_accelerations, 'r--', label='Y Acceleration', linewidth=2)
    axes[2].set_ylabel('Acceleration')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Ensure the outputs directory exists
    os.makedirs("outputs", exist_ok=True)
    
    # Save the figure
    plt.savefig(f"outputs/{filename_prefix}_profiles.png")
    plt.close(fig)