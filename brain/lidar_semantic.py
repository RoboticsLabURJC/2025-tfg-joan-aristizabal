import carla
import numpy as np
import open3d as o3d
from matplotlib import colormaps as cm
import math



VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

def set_lidar(world, ego_vehicle, delta):

    # Crear LiDAR
    lidar_transform = carla.Transform(carla.Location(z=2.5))
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
    lidar_bp.set_attribute('channels', '128')        # Número de rayos
    lidar_bp.set_attribute('range', '100')           # Alcance en metros
    lidar_bp.set_attribute('rotation_frequency', str(1 / delta))  # Hz
    lidar_bp.set_attribute('points_per_second', '1200000') 

    # Spawnear sensor y aplicarlo al vehículo
    sensor_lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
    return sensor_lidar
    
# Función para calcular la matriz de rotación
def euler_to_rotation_matrix(pitch, yaw, roll):

    # Convertir los ángulos de grados a radianes
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    roll = math.radians(roll)

    # Crear la matriz de rotación para cada eje
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Multiplicar en el orden Yaw -> Pitch -> Roll
    rotation_matrix = R_z @ R_y @ R_x
    return rotation_matrix

# Función de callback del sensor LiDAR
def lidar_callback(lidar_data, point_cloud, vehicle_transform, absolute_view):

    # Definir el formato del paquete de datos del LiDAR semántico
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('cos_inc_angle', np.float32),
        ('object_idx', np.uint32),
        ('object_tag', np.uint32)
    ])

    # Convertir los datos binarios a un array de numpy con el dtype estructurado
    data = np.frombuffer(lidar_data.raw_data, dtype=dtype)

    # Extraer coordenadas de los puntos
    points = np.vstack((data['x'], data['y'], data['z'])).T

    # Reflejar los datos en el eje X (para ajustarlos al sistema CARLA)
    points[:, 0] = -points[:, 0]

    # Obtener etiquetas semánticas (object_tag)
    semantic_tags = data['object_tag']

    # Asignar color según la etiqueta semántica
    # Por ejemplo, con el colormap inferno o viridis
    normalized_tags = (semantic_tags - semantic_tags.min()) / (np.ptp(semantic_tags) + 1e-6)
    colors = VIRIDIS[(normalized_tags * (VIRIDIS.shape[0]-1)).astype(np.int32)]

    if absolute_view:
        vehicle_location = np.array([
            -vehicle_transform.location.x,
            vehicle_transform.location.y,
            vehicle_transform.location.z
        ])

        pitch = vehicle_transform.rotation.pitch
        yaw = vehicle_transform.rotation.yaw
        roll = vehicle_transform.rotation.roll

        rotation_matrix = euler_to_rotation_matrix(pitch, yaw, roll).T

        # Transformar los puntos al sistema global
        points = (rotation_matrix @ points.T).T + vehicle_location

    # Actualizar el point cloud en Open3D
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
