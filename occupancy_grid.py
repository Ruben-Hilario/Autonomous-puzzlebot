import open3d as o3d
import numpy as np
import time
import matplotlib.pyplot as plt
import cv2 
import yaml

# Cargar el mesh desde un archivo PLY
mesh = o3d.io.read_triangle_mesh("pista.ply")  # Cambia el nombre si es necesario
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh])

# Obtener límites en Z
bbox = mesh.get_axis_aligned_bounding_box()
z_min, z_max = bbox.get_min_bound()[2], bbox.get_max_bound()[2]

# Umbrales de corte
z_lower = z_min + 0.11 * (z_max - z_min)  # Cortar el primer 10%
z_upper = z_min + 0.70 * (z_max - z_min)  # Cortar todo por encima del 70%

# Filtrar vértices
vertices = np.asarray(mesh.vertices)
triangles = np.asarray(mesh.triangles)

valid_z = np.logical_and(vertices[:, 2] >= z_lower, vertices[:, 2] <= z_upper)
valid_indices = np.where(valid_z)[0]
valid_index_set = set(valid_indices)

# Filtrar triángulos con todos los vértices válidos
filtered_triangles = [
    tri for tri in triangles
    if all(v_idx in valid_index_set for v_idx in tri)
]

# Crear nuevo mesh
filtered_vertices = vertices[valid_indices]
index_mapping = {old_idx: new_idx for new_idx, old_idx in enumerate(valid_indices)}

remapped_triangles = np.array([
    [index_mapping[v_idx] for v_idx in tri]
    for tri in filtered_triangles
])

new_mesh = o3d.geometry.TriangleMesh()
new_mesh.vertices = o3d.utility.Vector3dVector(filtered_vertices)
new_mesh.triangles = o3d.utility.Vector3iVector(remapped_triangles)
new_mesh.compute_vertex_normals()

# Mostrar resultado
o3d.visualization.draw_geometries([new_mesh])

# Voxelizar el mesh
voxel_size = 0.05  # Tamaño del voxel en metros (ajústalo según la escala de tu entorno)
voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(new_mesh, voxel_size=voxel_size)

# Extraer los voxeles ocupados
occupied_voxels = np.array([voxel.grid_index for voxel in voxel_grid.get_voxels()])

# Proyectar los voxeles al plano XY (ignoramos la altura Z)
xy_voxels = occupied_voxels[:, :2]

# Ajustar los índices para que sean positivos
xy_voxels -= np.min(xy_voxels, axis=0)

# Crear mapa 2D binario
grid_size = np.max(xy_voxels, axis=0) + 1
occupancy_grid = np.zeros(grid_size[::-1], dtype=np.uint8)  # (rows, cols) = (Y, X)

for x, y in xy_voxels:
    occupancy_grid[y, x] = 1  # nota: y es fila, x es columna

# Visualizar la grid
plt.imshow(occupancy_grid, cmap='gray')
plt.title("Occupancy Grid 2D")
plt.axis('equal')
plt.show()

# Invertir la grid para que 0 = libre, 100 = ocupado (estilo ROS)
ros_map = (1 - occupancy_grid) * 255  # 0 = ocupado, 255 = libre
ros_map = ros_map.astype(np.uint8)

# Guardar como imagen .pgm
cv2.imwrite("map.pgm", ros_map)

# Crear archivo .yaml
map_metadata = {
    "image": "map.pgm",
    "resolution": voxel_size,       # debe coincidir con el voxel_size
    "origin": [0.0, 0.0, 0.0],      # puedes ajustar si el mapa está desfasado
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.2
}

with open("map.yaml", "w") as f:
    yaml.dump(map_metadata, f, default_flow_style=False)
