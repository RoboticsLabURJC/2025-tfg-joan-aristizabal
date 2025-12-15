import carla
import pygame
import time
import math
import pandas as pd

# Variables globales
steer_real = 0.0          # ángulo real del volante
steer_rate = 0.0          # velocidad real del volante
last_time = time.time()
throttle = 0.0

tiempo = []
velocidad_giro = []

# Parámetros del volante
STEER_RATE_MAX = 1     # velocidad máxima de giro
ALPHA_STEER = 0.2      # suavizado

def control(vehicle):
    global steer_real, steer_rate, last_time, throttle

    control = carla.VehicleControl()
    keys = pygame.key.get_pressed()

    now = time.time()
    dt = now - last_time
    last_time = now

    # Velocidad objetivo según el giro izquierda(A)/derecha(D)
    if keys[pygame.K_a]:
        steer_rate_cmd = -STEER_RATE_MAX
    elif keys[pygame.K_d]:
        steer_rate_cmd = STEER_RATE_MAX
    else:
        steer_rate_cmd = 0.0
        steer_real = 0.0

    # Velocidad real de giro del volante
    steer_rate = (1 - ALPHA_STEER) * steer_rate + ALPHA_STEER * steer_rate_cmd


    # Cálculo de Steer
    steer_real += steer_rate * dt

    # Limitar ángulo de giro
    steer_real = max(-1.0, min(1.0, steer_real)) 
    control.steer = steer_real
   
    velocidad_giro.append(steer_rate)
    tiempo.append(now)

    # W throttle 
    if keys[pygame.K_w]:
        throttle = min(throttle + 0.25, 1.0)
    else:
        throttle = max(throttle - 0.25, 0.0)

    control.throttle = throttle

    # S freno
    if keys[pygame.K_s]:
        control.brake = 1.0
    else:
        control.brake = 0.0

    vehicle.apply_control(control)

    print(
        "| steer_rate:", round(steer_rate, 3),
        "| steer:", round(steer_real, 3),
        "| throttle:", round(control.throttle, 2)
    )


    return steer_rate, control.throttle, control.steer, control.brake