from typing import Tuple, Dict, List
import numpy as np
from numpy.typing import NDArray
from math import pi

from open3d.core import Tensor, float32
import open3d as o3d
from open3d.visualization import O3DVisualizer

# from utils import gen_spherical_gaussian_axis

def create_plane(width=640, height=480, color=(0,0,0), device=None):
    plane_pts = np.array([[x, y, 0] for x in range(width) for y in range(height)])
    plane_pts = plane_pts - [width//2, height//2, 0]
    
    if device is not None:
        pcd = o3d.t.geometry.PointCloud(device=device)
    else:
        pcd = o3d.t.geometry.PointCloud()
    pcd.point.positions = o3d.core.Tensor(plane_pts, o3d.core.float32)
    pcd.paint_uniform_color(Tensor([0.,0.,0.], float32))

    return pcd

def project_to_plane(plane_width: int, plane_height: int, 
        object_pcl: o3d.t.geometry.PointCloud, K: np.array,
        plane_pcl: o3d.t.geometry.PointCloud):
    # Project points to the camera at tangent plane plane_pcd and
    # visualize them as a white points on the plane.
    plane_colors = np.zeros((plane_width, plane_height, 3))
    plane_pts = np.dot(K, object_pcl.point.positions.numpy().T).T
    plane_pts = plane_pts//plane_pts[:, 2, None]
    inlier_mask = np.zeros(plane_pts.shape[0], dtype=bool)
    inlier_mask[
        (plane_pts[:, 0] > 0)*(plane_pts[:, 0] < plane_width)*(plane_pts[:, 1] > 0)*(plane_pts[:, 1] < plane_height)
        ] = True
    plane_colors[plane_pts[inlier_mask][:, 0].astype(int), plane_pts[inlier_mask][:, 1].astype(int)] = [
        1, 1, 1]
    plane_pcl.point.colors = o3d.core.Tensor(plane_colors.reshape(-1, 3, order='C'), o3d.core.float32)

def create_sphere_mesh(coords: tuple =(0,0,0), scale: float =1., radius=1.0):
    """
    Create TriangleMesh sphere and PBR materials.
    Return mesh and materials.
    """
    mat_sphere = o3d.visualization.rendering.MaterialRecord()
    # mat_sphere.shader = 'defaultLitTransparency'
    mat_sphere.shader = 'defaultLitSSR'
    mat_sphere.base_color = [0.467, 0.467, 0.467, 0.2]
    mat_sphere.base_roughness = 0.0
    mat_sphere.base_reflectance = 0.0
    mat_sphere.base_clearcoat = 1.0
    mat_sphere.thickness = 1.0
    mat_sphere.transmission = 1.0
    mat_sphere.absorption_distance = 100
    mat_sphere.absorption_color = [0.5, 0.5, 0.5]

    sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius, 
                        create_uv_map=True)
    # sphere_mesh = o3d.t.geometry.TriangleMesh.from_legacy(sphere_mesh)
    # sphere_mesh.compute_vertex_normals()
    sphere_mesh.translate(coords)

    # mat_sphere = convert_material_record(mat_record = mat_sphere)
    # sphere_mesh.material = mat_sphere
    # sphere_mesh.vertices = o3d.utility.Vector3dVector(
    #     (np.asarray(sphere_mesh.vertices) - np.asarray(sphere_mesh.vertices).mean(axis=0))*scale)
    
    return sphere_mesh, mat_sphere

def generate_rotation_matrices(initial_axis: NDArray, num_rotations: int) -> NDArray:

    rotations: NDArray = np.empty((num_rotations, 3, 3))
    axis = initial_axis
    
    for idx in range(num_rotations):
        axis = gen_spherical_gaussian_axis(old_axis=axis)
        angle = np.random.random(1)*2*np.pi/100
        
        rotations[idx] = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation=angle*axis)
        # rotation = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation=angle*axis)
        
        # yield rotation
        
    return rotations
        
    
def gen_spherical_gaussian_axis(old_axis):
    """
    Generate a random unit vector on the unit sphere close to the old_axis
    """
    axis = np.random.randn(3)
    axis /= np.linalg.norm(axis)
    axis = axis * 0.1 + old_axis * 0.9
    axis /= np.linalg.norm(axis)
    return axis
        
def rotate_axis_angle(old_axis: NDArray, pcl: o3d.t.geometry.PointCloud):

    axis = gen_spherical_gaussian_axis(old_axis=old_axis)
    axis = axis/np.linalg.norm(axis)
    angle = np.random.random(1)*2*np.pi/100
    print(f"axis: {axis}, angle: {angle}")

    pcl.rotate(
        R=o3d.geometry.get_rotation_matrix_from_axis_angle(rotation=angle*axis),
        center=[0,0,0]
    )
    
    return pcl