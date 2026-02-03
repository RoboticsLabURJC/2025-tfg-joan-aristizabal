import carla
from lidar_semantic import set_lidar
from lidar_semantic import lidar_callback
from vehicle_control import control
from camera import set_camera
from pilotnet import load_pilotnet
import pygame
import open3d as o3d
from datetime import datetime
import sys
import random
import numpy as np
import cv2
import torch
import torchvision.transforms as T
import numpy as np

actor_list = []         # Lista de actores de Carla
labels = []             # Lista de etiquetas
absolute_view = False   # Vista del LiDAR
current_view_index = 0  # Indice de la vista actual
name = ""


# Actuadores del vehículo
throttle = 1    # acelerador
steer = 0       # giro
brake = 0       # freno

# Tipos de vistas de visualización del sensor LiDAR
camera_views = [
    {"zoom": 0.3, "front": [0, 0, 1], "lookat": [0, 0, 0], "up": [-1, 0, 0], "view":"Primera persona"},        # Primera persona
    {"zoom": 0.06, "front": [1.0, 0.0, 0.3], "lookat": [0, 0, 0], "up": [0, 0, 1], "view":"Tercera persona"},  # Tercera persona
    {"zoom": 0.2, "front": [0, 0, -1], "lookat": [0, 0, 0], "up": [0, 1, 0], "view":"Vista cenital"}           # Vista cenital
]

device = "cuda" if torch.cuda.is_available() else "cpu"
model = load_pilotnet("modelo/pilotnet_steer_throttle_best_error.pt", device)


IMG_WIDTH = 160
IMG_HEIGHT = 120

# Preprocesar frames para pasar al modelo de predicción
preprocess = T.Compose([
    T.Resize((120, 160)),
    T.ToTensor(),  # -> (3,H,W) en [0,1]
])

@torch.no_grad()
def predict_controls(model, img_rgb_np, device="cpu"):
    """
    img_rgb_np: numpy array RGB uint8 (H,W,3)
    devuelve: (steer, throttle)
      - steer ∈ [-1,1]
      - throttle ∈ [0,1]
    """

    # Resize
    img = cv2.resize(img_rgb_np, (IMG_WIDTH, IMG_HEIGHT))

    # Normalizar a [0,1]
    img = img.astype(np.float32) / 255.0

    # HWC -> CHW
    img = np.transpose(img, (2, 0, 1))

    # Añadir batch dim: (1,3,H,W)
    x = torch.from_numpy(img).unsqueeze(0).to(device)

    # Forward
    y = model(x)          # tensor (1,2) -> [steer, throttle]

    steer = float(y[0, 0].item())
    throttle = float(y[0, 1].item())

    return steer, throttle

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

    
    frame_name = f"{image.frame:06d}.png"
   
    global steer, throttle, model, name
    name = frame_name
    print(frame_name)
    steer, throttle = predict_controls(model, array_rgb)

# Crear una esfera en el origen: donde está el sensor/vehículo
def create_origin_sphere():
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=3)
    sphere.translate([0, 0, 0])  # Posicionar en el origen
    sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Color rojo
    return sphere

# Ajustar la vista seleccionada del diccionario "camera_views"
def set_camera_view(viz,view_index):
    ctr = viz.get_view_control()
    view = camera_views[view_index]
    ctr.set_zoom(view["zoom"])
    ctr.set_front(view["front"])
    ctr.set_lookat(view["lookat"])
    ctr.set_up(view["up"])
    print(f'Cámara ajustada: {view["view"]}')

# Cerebro / Función principal
def brain():

    pygame.init()

    # Configurar Pygame 
    pygame.display.gl_set_attribute(pygame.GL_ACCELERATED_VISUAL, 0)
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height), pygame.SRCALPHA)
    pygame.display.set_caption("CARLA Vehículo Control")

    # Conexión con servidor de Carla
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    #client.load_world('Town02') #Establecer escenario determinado

    # Crear escenario
    world = client.get_world()

    # Librería del simulador
    blueprint_library = world.get_blueprint_library()
    traffic_manager = client.get_trafficmanager(8000)               # Control del tráfico de vehículos automáticos
    traffic_manager.set_synchronous_mode(True)                      # Modo síncrono: avanza en cada "world.tick()"
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)     # Distancia de seguridad entre vehículos

    # Configuración del simulador
    settings = world.get_settings()                # Obtener configuración actual              
    delta = 0.033                                   # 0.033 s por cada tick: 30 FPS
    settings.fixed_delta_seconds = delta           # la simulación avanza cada 0.033 s
    settings.synchronous_mode = True               # Activa el modo síncrono
    world.apply_settings(settings)                 # Aplica los ajustes modificados

    
    # Librería de vehículos
    vehicle_blueprints = blueprint_library.filter('*vehicle*')

    # Cogemos todos los puntos de aparición predefinidos del escenario
    spawn_points = world.get_map().get_spawn_points()

    # Seleccionamos un vehículo determinado de la librería
    vehicle_bp = vehicle_blueprints.find('vehicle.audi.tt')

    z = random.randint(1,len(spawn_points))
    # Generar vehículo con el que se va a trabajar
    ego_vehicle = world.spawn_actor(vehicle_bp, spawn_points[z])
    if ego_vehicle is not None:
        print('Vehículo spawneado correctamente')

    # Establecer sensor LiDAR al vehículo
    lidar = set_lidar(world, ego_vehicle, delta)

    # Establecer cámara rgb al vehículo
    camera = set_camera(world, ego_vehicle)
    
    global actor_list, current_view_index, origin_sphere_added, absolute_view
    actor_list.append(ego_vehicle)
    actor_list.append(camera)
    actor_list.append(lidar)
    
    # Llamar al callback de la cámara
    camera.listen(lambda image: camera_callback(image, screen))

    # Crear nube de puntos y llamar al callback del LiDAR
    point_cloud = o3d.geometry.PointCloud()
    lidar.listen(lambda data: lidar_callback(data, point_cloud, lidar.get_transform(), absolute_view))

    # Visualizador de Open3D y configuraciones iniciales
    viz = o3d.visualization.VisualizerWithKeyCallback()
    viz.create_window(window_name='Sensor LiDAR', width=960, height=540, left=480, top=270)
    viz.get_render_option().background_color = [0.05, 0.05, 0.05]
    viz.get_render_option().point_size = 1.35
    viz.get_render_option().show_coordinate_frame = True

    # Configurar vista
    set_camera_view(viz,current_view_index)

    #Inicializar efera en el origen
    origin_sphere = create_origin_sphere()
    origin_sphere_added = False

    def toggle_camera_view(_):
        global current_view_index, absolute_view, origin_sphere_added
        # Alternar a la siguiente vista en el array de cámaras
        current_view_index = (current_view_index + 1) % len(camera_views)
        set_camera_view(viz, current_view_index)
        
        # Activar o desactivar vista absoluta y la esfera en el origen
        absolute_view = (current_view_index == len(camera_views) - 1)  # Vista cenital como absoluta
        if absolute_view and not origin_sphere_added:
            viz.add_geometry(origin_sphere)
            origin_sphere_added = True
        elif not absolute_view and origin_sphere_added:
            viz.remove_geometry(origin_sphere)
            origin_sphere_added = False
        
        print(f"Cambiando a vista {current_view_index + 1}")
        return True

    # Callback para alternar vistas
    viz.register_key_callback(ord("V"), toggle_camera_view)

    global throttle, steer

    # Ciclo principal de visualización
    lidar_data_received = False
    dt0 = datetime.now()
    frame = 0
    try:
        while True:
            global steer,throttle, name
            # Procesar control del vehículo
            control(ego_vehicle, steer, throttle, name)

            # Usar snapshot frame para sincronizar labels con los sensores
            snapshot = world.get_snapshot()
            frame_id = snapshot.frame

            # Procesar eventos de Pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print("\n Cierre de ventana detectado.")
                    raise KeyboardInterrupt

            # Añadir nube de puntos al visualizador una sola vez
            if frame == 5 and not lidar_data_received:
                viz.add_geometry(point_cloud)
                lidar_data_received = True
                print("Nube de puntos añadida a Open3D")

            # Actualizar visualización
            viz.update_geometry(point_cloud)    # actualizar nube de puntos
            viz.poll_events()                   # procesar eventos del sistema de ventanas
            viz.update_renderer()               # actualizar el contenido de la ventana
            world.tick()                        # avanzar la simulación


            # Mostrar FPS
            process_time = datetime.now() - dt0
            if process_time.total_seconds() > 0:
                fps = 1.0 / process_time.total_seconds()
                sys.stdout.write(f'\rFPS: {fps:.2f} ')
                sys.stdout.flush()
            dt0 = datetime.now()
            frame += 1

            # Salir si Open3D se cierra
            if not viz.poll_events():
                break

    except KeyboardInterrupt:
        pass
    finally:
        cleanup()

# Función de limpieza de actores del simulador (vehículos, sensores,etc)
def cleanup():
    global actor_list
    
    # Limpieza de actores en Carla
    print("\nLimpiando actores...")
    for actor in actor_list:
        if actor is not None:
            actor.destroy()
    actor_list = []
    print("Actores eliminados.")

# Ejecución del programa principal
brain()