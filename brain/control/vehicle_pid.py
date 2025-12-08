import carla
import pygame
import time
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def update(self, measurement):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        error = self.setpoint - measurement

        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        D = self.Kd * ((error - self.prev_error) / dt) if dt > 0 else 0

        self.prev_error = error
        return P + I + D


speed_pid = PID(Kp=0.05, Ki=0.01, Kd=0.02)

MAX_SETPOINT = 50.0  # km/h
SETPOINT_INCREMENT = 1.0  # cómo sube al mantener W
SETPOINT_DECREMENT = 1.0  # cómo baja al soltar W


def get_speed(vehicle):
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)


def control(vehicle):
    control = carla.VehicleControl()
    keys = pygame.key.get_pressed()

    # Setpoint W
    if keys[pygame.K_w]:
        speed_pid.setpoint += SETPOINT_INCREMENT
    else:
        speed_pid.setpoint -= SETPOINT_DECREMENT

    speed_pid.setpoint = max(0.0, min(MAX_SETPOINT, speed_pid.setpoint))

    # Velocidad medida
    current_speed = get_speed(vehicle)

    # Throttle
    throttle_cmd = speed_pid.update(current_speed)
    throttle_cmd = max(0.0, min(1.0, throttle_cmd))

    control.throttle = throttle_cmd

    # S stop
    if keys[pygame.K_s]:
        control.brake = 1.0
    else: 
        control.brake = 0.0
    
    # A girar a la izquierda
    if keys[pygame.K_a]:
        control.steer = -0.5

    # D : girar a la derecha
    if keys[pygame.K_d]:
        control.steer = 0.5

    vehicle.apply_control(control)

    print(f"Speed={current_speed:.1f} km/h | Throttle={control.throttle:.2f} | Setpoint={speed_pid.setpoint:.1f}")

    return control.throttle, control.steer, control.brake
