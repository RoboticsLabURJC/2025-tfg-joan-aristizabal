import carla
import pygame
import numpy as np
import math

throttle = 1.0
steer = 0.0
brake = 0.0


# Función que permite el control del vehículo por teclado
def control(vehicle, predicted_steer, predicted_throttle, name):

    control = carla.VehicleControl()
    
    # Leer teclas
    keys = pygame.key.get_pressed()

    global steer, throttle, brake
    steer = predicted_steer
    throttle = predicted_throttle

    # valores iniciales
    control.steer = steer
    control.throttle = throttle

    # S stop
    if keys[pygame.K_s]:
        control.brake = 1.0

    # Aplicar al vehículo
    vehicle.apply_control(control)
    print(f'Frame{name}: ',control.steer, control.throttle, control.brake)

    return control.throttle, control.steer, control.brake

