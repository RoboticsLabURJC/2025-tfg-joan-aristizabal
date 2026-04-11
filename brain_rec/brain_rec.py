import carla
from lidar_semantic import set_lidar
from lidar_semantic import lidar_callback
from vehicle_steer_2 import control
from camera_rec import set_camera
from camera_rec import camera_callback
import pygame
import open3d as o3d
from datetime import datetime
import sys
import time
import os
import pandas as pd
import random

actor_list = []         # Lista de actores de Carla
labels = []             # Lista de etiquetas
absolute_view = False   # Vista del LiDAR
current_view_index = 0  # Indice de la vista actual


# Actuadores del vehículo
throttle = 0    # acelerador
steer = 0       # giro
brake = 0       # freno
steer_rate = 0  # velocidad de giro
name = ""       # Nombre de la etiqueta
start_id = 39984
# Tipos de vistas de visualización del sensor LiDAR
camera_views = [
    {"zoom": 0.3, "front": [0, 0, 1], "lookat": [0, 0, 0], "up": [-1, 0, 0], "view":"Primera persona"},        # Primera persona
    {"zoom": 0.06, "front": [1.0, 0.0, 0.3], "lookat": [0, 0, 0], "up": [0, 0, 1], "view":"Tercera persona"},  # Tercera persona
    {"zoom": 0.2, "front": [0, 0, -1], "lookat": [0, 0, 0], "up": [0, 1, 0], "view":"Vista cenital"}           # Vista cenital
]


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

# Añadir vehículos secundarios al escenario
def adding_npcs(world, spawn_points):
    
    vehicles =[]
    global actor_list
    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    spawn_attempts = 0

    for i in range(100):
            blueprint = random.choice(vehicle_blueprints)
            spawn_point = random.choice(spawn_points)

            vehicle = world.try_spawn_actor(blueprint, spawn_point)
            spawn_attempts += 1

            if vehicle is not None:
                vehicles.append(vehicle)
                actor_list.append(vehicle)
                vehicle.set_autopilot(True)
                print(f"NPC {i+1} añadido en {spawn_point.location}")
            else:
                print(f"Fallo al spawnear vehículo {i+1}, punto ocupado o inválido.")

    print(f"\nIntentos totales: {spawn_attempts} | NPCs añadidos: {len(vehicles)}")

# Cerebro / Función principal
def brain():

    pygame.init()

    # Configurar Pygame sin usar OpenGL explícitamente
    pygame.display.gl_set_attribute(pygame.GL_ACCELERATED_VISUAL, 0)
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height), pygame.SRCALPHA)
    pygame.display.set_caption("CARLA Vehículo Control")

    # Conexión con servidor de Carla
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    #client.load_world('Town03') #Establecer escenario determinado

    # Crear escenario
    world = client.get_world()

    # Librería del simulador
    blueprint_library = world.get_blueprint_library()
    traffic_manager = client.get_trafficmanager(8000)               # Control del tráfico de vehículos automáticos
    traffic_manager.set_synchronous_mode(True)                      # Modo síncrono: avanza en cada "world.tick()"
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)     # Distancia de seguridad entre vehículos

    # Configuración del simulador
    settings = world.get_settings()                # Obtener configuración actual              
    delta = 0.033                                   # 0.05 s por cada tick: 20 FPS
    settings.fixed_delta_seconds = delta           # la simulación avanza cada 0.05 s
    settings.synchronous_mode = True               # Activa el modo síncrono
    world.apply_settings(settings)                 # Aplica los ajustes modificados

    
    # Librería de vehículos
    vehicle_blueprints = blueprint_library.filter('*vehicle*')

    # Cogemos todos los puntos de aparición predefinidos del escenario
    spawn_points = world.get_map().get_spawn_points()

    # Seleccionamos un vehículo determinado de la librería
    vehicle_bp = vehicle_blueprints.find('vehicle.audi.tt')

    # Generar vehículo con el que se va a trabajar
    z = random.randint(1,len(spawn_points))
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

    # Añadir NPCs al escenario
    #adding_npcs(world, spawn_points)
    
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

    global labels, throttle, steer, brake, start_id, steer_rate
    # Ciclo principal de visualización
    lidar_data_received = False
    dt0 = datetime.now()
    frame = 0
    frame_id = start_id

    try:
        while True:

            # Procesar control del vehículo
            steer_rate, throttle, steer, brake = control(ego_vehicle)
             # Usar snapshot frame para sincronizar labels con los sensores
            
            frame_name = f"{frame_id:06d}"
            frame_id += 1

            #snapshot = world.get_snapshot()
            #frame_id = snapshot.frame

            # Etiquetas para cadad frame capturado
            labels.append({
                'frame_id': frame_name,
                'steer_rate': steer_rate,
                'steer': steer,
                'throttle': throttle,
                'brake': brake,
            })

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
            time.sleep(0.03)
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
    global actor_list, labels

    # Almacenar las features de las capturas al finalizar
    df = pd.DataFrame(labels)
    output_dir = 'dataset_2'
    os.makedirs(output_dir, exist_ok=True)
    df.to_csv(os.path.join(output_dir, 'labels.csv'), index=False)

    # Limpieza de actores en Carla
    print("\nLimpiando actores...")
    for actor in actor_list:
        if actor is not None:
            actor.destroy()
    actor_list = []
    print("Actores eliminados.")

# Ejecución del programa principal
brain()