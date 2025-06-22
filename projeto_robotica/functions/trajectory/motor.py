class MotorLimits:
    """Limites físicos do motor"""
    max_velocity: float  # velocidade máxima [rad/s]
    max_acceleration: float  # aceleração máxima [rad/s²]

    def __init__(self, max_velocity: float, max_acceleration: float):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration