# import roboticstoolbox as rtb
# import numpy as np

# from roboticstoolbox import DHRobot, RevoluteDH

# # Robô planar 2 DOF
# link1 = RevoluteDH(a=1.0, alpha=0)
# link2 = RevoluteDH(a=1.0, alpha=0)
# robot = DHRobot([link1, link2], name="Planar2DOF")

# # Trajetória exemplo: acelerar, cruzeiro, desacelerar
# q_start = np.array([0, 0])
# q_end = np.array([np.pi/4, np.pi/3])

# q_accel = rtb.jtraj(q_start, (q_start + q_end)/2, 20)
# q_cruise = rtb.jtraj((q_start + q_end)/2, (q_start + q_end)/2 + (q_end - q_start)/4, 40)
# q_decel = rtb.jtraj((q_start + q_end)/2 + (q_end - q_start)/4, q_end, 20)

# q_full = np.vstack([q_accel.q, q_cruise.q, q_decel.q])
# # print(q_full)

# # ✅ 2D Plot! — ideal para plano XY
# robot.plot(
#     q_full,
#     dt=0.05,
#     backend="pyplot",
#     movie="RR.gif",
#     block=False,
# )

# ------------------------------------------------------------------------------------------------------------------------------- #

# import numpy as np
# import matplotlib.pyplot as plt

# # 2) Modifique a função para aceitar um vetor de tempo opcional
# def trapezoidal_profile(q0, qf, V_max, A_max, t=None):
#     D = qf - q0
#     sgn = np.sign(D)
#     D = abs(D)
#     V_max = abs(V_max)
#     A_max = abs(A_max)

#     # Tempo para acelerar até V_max
#     t_acc = V_max / A_max
#     D_acc = 0.5 * A_max * t_acc ** 2

#     if 2 * D_acc >= D:
#         t_acc = np.sqrt(D / A_max)
#         t_flat = 0.0
#         T = 2 * t_acc
#     else:
#         D_flat = D - 2 * D_acc
#         t_flat = D_flat / V_max
#         T = 2 * t_acc + t_flat

#     if t is None:
#         dt = 0.01
#         t = np.arange(0, T + dt, dt)
#     else:
#         T = t[-1]

#     q = np.zeros_like(t)

#     for i, ti in enumerate(t):
#         if ti < t_acc:
#             q[i] = q0 + sgn * 0.5 * A_max * ti**2
#         elif ti < t_acc + t_flat:
#             q[i] = q0 + sgn * (D_acc + V_max * (ti - t_acc))
#         elif ti <= T:
#             q[i] = qf - sgn * 0.5 * A_max * (T - ti)**2

#     return t, q, T

# # -------- PARÂMETROS --------

# theta1, theta2 = 0.0, 0.0
# theta3, theta4 = np.pi, np.pi/2

# V_max = np.pi/4  # Velocidade máxima (rad/s)
# A_max = np.pi/8  # Aceleração máxima (rad/s^2)

# # -------- GERA PERFIS INDIVIDUAIS --------

# t1, q1, T1 = trapezoidal_profile(theta1, theta3, V_max, A_max)
# t2, q2, T2 = trapezoidal_profile(theta2, theta4, V_max, A_max)

# # Crie t_sync uma vez
# dt = 0.01
# T_sync = max(T1, T2)
# t_sync = np.arange(0, T_sync + dt, dt)

# # Passe t_sync para ambas as juntas
# _, q1_sync, _ = trapezoidal_profile(theta1, theta3, V_max, A_max, t=t_sync)
# _, q2_sync, _ = trapezoidal_profile(theta2, theta4, V_max, A_max, t=t_sync)

# # Calcula dt real:
# dt = t_sync[1] - t_sync[0]

# # Velocidade = dq/dt
# q1d = np.gradient(q1_sync, dt)
# q2d = np.gradient(q2_sync, dt)

# # Aceleração = d²q/dt²
# q1dd = np.gradient(q1d, dt)
# q2dd = np.gradient(q2d, dt)


# fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

# # Posição
# # axs[0].plot(t_sync, q1_sync, label="Junta 1")
# axs[0].plot(t_sync, q2_sync, label="Junta 2")
# axs[0].set_ylabel("Ângulo [rad]")
# axs[0].legend()
# axs[0].grid(True)

# # Velocidade
# # axs[1].plot(t_sync, q1d, label="Junta 1")
# axs[1].plot(t_sync, q2d, label="Junta 2")
# axs[1].set_ylabel("Velocidade [rad/s]")
# axs[1].legend()
# axs[1].grid(True)

# # Aceleração
# # axs[2].plot(t_sync, q1dd, label="Junta 1")
# axs[2].plot(t_sync, q2dd, label="Junta 2")
# axs[2].set_ylabel("Aceleração [rad/s²]")
# axs[2].set_xlabel("Tempo [s]")
# axs[2].legend()
# axs[2].grid(True)

# plt.tight_layout()
# plt.show()

# ------------------------------------------------------------------------------------------------------------------------------- #

# import numpy as np

# def trapezoidal_profile_sync(q0, qf, A_max, T, t=None):
#     D = qf - q0
#     sgn = np.sign(D)
#     D = abs(D)
#     A_max = abs(A_max)

#     # Resolve t_acc usando fórmula inversa
#     delta = T**2 - 4 * D / A_max
#     if delta < 0:
#         raise ValueError("Impossível sincronizar com essa aceleração: T muito curto ou A_max muito pequeno.")
#     t_acc = 0.5 * (T - np.sqrt(delta))
#     V_max = A_max * t_acc
#     t_flat = T - 2 * t_acc

#     # Cria vetor de tempo
#     if t is None:
#         dt = 0.01
#         t = np.arange(0, T + dt, dt)

#     q = np.zeros_like(t)

#     for i, ti in enumerate(t):
#         if ti < t_acc:
#             q[i] = q0 + sgn * 0.5 * A_max * ti**2
#         elif ti < t_acc + t_flat:
#             q[i] = q0 + sgn * (0.5 * A_max * t_acc**2 + V_max * (ti - t_acc))
#         elif ti <= T:
#             t_dec = ti - (t_acc + t_flat)
#             q[i] = qf - sgn * 0.5 * A_max * (T - ti)**2

#     return t, q, T, V_max

# # Parâmetros
# theta1, theta2 = 0.0, 0.0
# theta3, theta4 = np.pi/2, np.pi/4

# A_max = np.pi/8
# T_sync = 4.0  # escolha o tempo desejado para ambos

# # Tempo de amostragem
# dt = 0.01
# t_sync = np.arange(0, T_sync + dt, dt)

# # Tempo mínimo teórico para cada junta
# D1 = abs(theta3 - theta1)
# D2 = abs(theta4 - theta2)
# Tmin1 = np.sqrt(4 * D1 / A_max)
# Tmin2 = np.sqrt(4 * D2 / A_max)
# T_min_required = max(Tmin1, Tmin2)

# print(f"T mínimo teórico necessário: {T_min_required:.2f} s")

# # Ajuste se precisar
# if T_sync < T_min_required:
#     print(f"Ajustando T_sync de {T_sync:.2f} s para {T_min_required:.2f} s")
#     T_sync = T_min_required + 0.1  # adiciona uma folga de segurança

# # Junta 1 — calcula V_max automático!
# _, q1_sync, _, V1 = trapezoidal_profile_sync(theta1, theta3, A_max, T_sync, t=t_sync)
# print(f"Junta 1 V_max ajustada: {V1:.3f} rad/s")

# # Junta 2 — idem
# _, q2_sync, _, V2 = trapezoidal_profile_sync(theta2, theta4, A_max, T_sync, t=t_sync)
# print(f"Junta 2 V_max ajustada: {V2:.3f} rad/s")

# # Deriva se quiser
# q1d = np.gradient(q1_sync, dt)
# q2d = np.gradient(q2_sync, dt)
# q1dd = np.gradient(q1d, dt)
# q2dd = np.gradient(q2d, dt)

# import matplotlib.pyplot as plt

# # --- Subplots organizados ---
# fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# # --- Posição ---
# axs[0].plot(t_sync, q1_sync, label="Junta 1")
# axs[0].plot(t_sync, q2_sync, label="Junta 2")
# axs[0].set_ylabel("Ângulo [rad]")
# axs[0].set_title("Posição")
# axs[0].legend()
# axs[0].grid(True)

# # --- Velocidade ---
# axs[1].plot(t_sync, q1d, label="Junta 1")
# axs[1].plot(t_sync, q2d, label="Junta 2")
# axs[1].set_ylabel("Velocidade [rad/s]")
# axs[1].set_title("Velocidade")
# axs[1].legend()
# axs[1].grid(True)

# # --- Aceleração ---
# axs[2].plot(t_sync, q1dd, label="Junta 1")
# axs[2].plot(t_sync, q2dd, label="Junta 2")
# axs[2].set_ylabel("Aceleração [rad/s²]")
# axs[2].set_xlabel("Tempo [s]")
# axs[2].set_title("Aceleração")
# axs[2].legend()
# axs[2].grid(True)

# plt.tight_layout()
# plt.show()

# ------------------------------------------------------------------------------------------------------------------------------- #

import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# NOVA FUNÇÃO
# -----------------------------
def trapezoidal_profile_fixed_fraction(q0, qf, T, cruise_fraction, t=None):
    D = qf - q0
    sgn = np.sign(D)
    D = abs(D)

    t_flat = cruise_fraction * T
    t_acc = 0.5 * (T - t_flat)

    # Calcular A_max que faz bater o deslocamento
    A_max = D / ( t_acc * (t_acc + t_flat) )

    V_max = A_max * t_acc

    # Verificação de consistência
    D_acc = 0.5 * A_max * t_acc**2
    D_flat = V_max * t_flat
    D_total = 2 * D_acc + D_flat

    if abs(D_total - D) > 1e-6:
        raise ValueError(
            f"Erro interno: deslocamento calculado {D_total:.4f} difere de desejado {D:.4f}"
        )

    if t is None:
        dt = 0.01
        t = np.arange(0, T + dt, dt)

    q = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < t_acc:
            q[i] = q0 + sgn * 0.5 * A_max * ti**2
        elif ti < t_acc + t_flat:
            q[i] = q0 + sgn * (D_acc + V_max * (ti - t_acc))
        elif ti <= T:
            q[i] = qf - sgn * 0.5 * A_max * (T - ti)**2

    return t, q, T, A_max, V_max

# -----------------------------
# PARÂMETROS
# -----------------------------
theta1, theta2 = 0.0, 0.0
theta3, theta4 = np.pi/2, np.pi/8

A_max = np.pi/4
T_sync = 2.0
cruise_fraction = 0.5  # 50% do tempo total em cruzeiro

dt = 0.01

# Tempo mínimo teórico usando a fração de cruzeiro
D1 = abs(theta3 - theta1)
D2 = abs(theta4 - theta2)

def T_min_for_fraction(D, A_max, cruise_fraction):
    # Resolve inverso:
    # D = A_max * t_acc^2 + A_max * t_acc * t_flat
    # t_flat = cruise_fraction * T
    # t_acc = 0.5 * (T - t_flat)
    # Substituindo:
    # D = A_max * t_acc^2 + A_max * t_acc * t_flat
    # tudo em função de T

    # Deixa em função de T:
    f = cruise_fraction
    T = np.sqrt(4 * D / (A_max * (1 - f)**2))
    return T

Tmin1 = T_min_for_fraction(D1, A_max, cruise_fraction)
Tmin2 = T_min_for_fraction(D2, A_max, cruise_fraction)
T_min_required = max(Tmin1, Tmin2)

# print(f"T mínimo com {cruise_fraction*100:.0f}% de cruzeiro: {T_min_required:.3f} s")

if T_sync < T_min_required:
    # print(f"Ajustando T_sync de {T_sync:.3f} para {T_min_required:.3f}")
    T_sync = T_min_required + 0.1

t_sync = np.arange(0, T_sync + dt, dt)

# Junta 1
_, q1_sync, _, A1, V1 = trapezoidal_profile_fixed_fraction(theta1, theta3, T_sync, cruise_fraction, t=t_sync)
# print(f"Junta 1 A_max ajustada: {A1:.3f} rad/s² | V_max: {V1:.3f} rad/s")

# Junta 2
_, q2_sync, _, A2, V2 = trapezoidal_profile_fixed_fraction(theta2, theta4, T_sync, cruise_fraction, t=t_sync)
# print(f"Junta 2 A_max ajustada: {A2:.3f} rad/s² | V_max: {V2:.3f} rad/s")

# Derivadas
q1d = np.gradient(q1_sync, dt)
q2d = np.gradient(q2_sync, dt)
q1dd = np.gradient(q1d, dt)
q2dd = np.gradient(q2d, dt)

# -----------------------------
# PLOT
# -----------------------------
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

axs[0].plot(t_sync, q1_sync, label="Junta 1")
axs[0].plot(t_sync, q2_sync, label="Junta 2")
axs[0].set_ylabel("Ângulo [rad]")
axs[0].set_title("Posição")
axs[0].legend()
axs[0].grid(True)

axs[1].plot(t_sync, q1d, label="Junta 1")
axs[1].plot(t_sync, q2d, label="Junta 2")
axs[1].set_ylabel("Velocidade [rad/s]")
axs[1].set_title("Velocidade")
axs[1].legend()
axs[1].grid(True)

axs[2].plot(t_sync, q1dd, label="Junta 1")
axs[2].plot(t_sync, q2dd, label="Junta 2")
axs[2].set_ylabel("Aceleração [rad/s²]")
axs[2].set_xlabel("Tempo [s]")
axs[2].set_title("Aceleração")
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.show()