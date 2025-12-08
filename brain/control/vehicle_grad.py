import carla
import pygame
import time

throttle = 0.0
steer = 0.0

last_step_time = time.time()
STEP_INTERVAL = 0.5     # tiempo entre incrementos en segundos
THROTTLE_STEP = 0.25    # tamaño del incremento

def control(vehicle):
    global throttle, steer, last_step_time

    control = carla.VehicleControl()
    keys = pygame.key.get_pressed()

    now = time.time()

    # W acelerar // Va en función del tiempo, para hacer más gradual la subida y caída del acelerador
    if keys[pygame.K_w] and (now - last_step_time) >= STEP_INTERVAL:
        throttle = min(throttle + THROTTLE_STEP, 1.0)
        last_step_time = now

    elif not keys[pygame.K_w] and (now - last_step_time) >= STEP_INTERVAL:
        throttle = max(throttle - THROTTLE_STEP, 0.0)
        last_step_time = now

    control.throttle = throttle

    # S stop
    if keys[pygame.K_s]:
        control.brake = 1.0
    
    # A girar a la izquierda
    if keys[pygame.K_a]:
        control.steer = -0.5

    # D : girar a la derecha
    if keys[pygame.K_d]:
        control.steer = 0.5

    vehicle.apply_control(control)

    print(f"STEER={control.steer:.2f} | THROTTLE={control.throttle:.2f} | BRAKE={control.brake:.2f}")

    return control.throttle, control.steer, control.brake
