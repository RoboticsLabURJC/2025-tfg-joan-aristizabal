import carla
import numpy as np
import pygame
import cv2
import os
import tensorflow as tf

IMG_WIDTH = 160
IMG_HEIGHT = 120
start_id = 39984

# Función para establecer cámara en el vehículo
def set_camera(world, vehicle):
    
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_transform = carla.Transform(
        carla.Location(x=1.5, z=1.2),       # 1.5 m adelante, 1.2 m de altura
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))  # mirando hacia adelante
    
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    return camera

# Función para procesar imágenes de la cámara
def camera_callback(image, display):

    # VISUALIZACIÓN
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (image.height, image.width, 4))
    array_bgra = array[:, :, :3]  # quitar canal alfa
    array_rgb = array_bgra[:, :, ::-1]  # BGRA → RGB
    surface = pygame.surfarray.make_surface(array_rgb.swapaxes(0, 1))
    display.blit(surface, (0, 0))
    pygame.display.update(display.get_rect())

    # CAPTURA DE DATOS
    global start_id
    output_dir = 'dataset_2/images'
    os.makedirs(output_dir, exist_ok=True)
    frame_id = start_id
    frame_name = f"{frame_id:06d}.png"
    cv2.imwrite(os.path.join(output_dir, frame_name), array_bgra)
    #print(f"Frame saved: {frame_name}")

    start_id += 1
    #steer, throttle, brake = predict_control(array_rgb)
    #print(frame_name, steer, throttle, brake )

    
