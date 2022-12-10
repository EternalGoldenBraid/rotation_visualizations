import open3d as o3d
import numpy as np

from numpy.typing import NDArray

def create_sphere_pcl(n_pts_sphere=100, coords=(0,0,0)):

    sphere_pts = []
    for i in range(n_pts_sphere):
        for j in range(n_pts_sphere):
            x = 100*np.cos(np.pi * i / 100) * np.cos(2 * np.pi * j / 100)
            y = 100*np.cos(np.pi * i / 100) * np.sin(2 * np.pi * j / 100)
            z = 100*np.sin(np.pi * i / 100)
            sphere_pts.append([x, y, z])
    sphere_pcd = o3d.geometry.PointCloud()
    sphere_pcd.points = o3d.utility.Vector3dVector(sphere_pts)
    sphere_pcd.translate(coords, relative=True)
    
    return sphere_pcd

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

    sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere_mesh.compute_vertex_normals()
    sphere_mesh.translate(coords)
    # sphere_mesh.vertices = o3d.utility.Vector3dVector(
    #     (np.asarray(sphere_mesh.vertices) - np.asarray(sphere_mesh.vertices).mean(axis=0))*scale)
    
    return sphere_mesh, mat_sphere

def setup_canvas(width=640, height=480, color=(0,0,0)):
    plane_pts = np.array([[x, y, 0] for x in range(width) for y in range(height)])
    plane_pts = plane_pts - [width//2, height//2, 0]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(plane_pts)

    return pcd

def get_axis_angle_from_matrix(R: np.array):

    angle = np.arccos((np.trace(R)-1)/2)
    u = 1/(2*np.sin(angle))*(R-R.T) # https://thenumb.at/Exponential-Rotations/
    
    return (u, angle)

import math
def get_rotating_transform(u, v):
    """
    Given two unit vectors u, v,
    we compute the rotation R such that v = R @ u
    """
    assert np.isclose(np.linalg.norm(u), 1)
    assert np.isclose(np.linalg.norm(v), 1)
    n = np.cross(u, v)
    while np.allclose(n, 0): # u and v are on the same line -.-
        rand_dir = np.random.randn(3)
        n = np.cross(u, rand_dir)
    n /= np.linalg.norm(n)
    t = np.cross(n, u)
    T = np.stack([u, t, n], axis=1)
    alpha = math.atan2(v @ t, v @ u)
    R = np.array([
        [math.cos(alpha), -math.sin(alpha), 0],
        [math.sin(alpha), math.cos(alpha), 0],
        [0, 0, 1]
    ])
    transform = T @ R @ T.T
    assert np.allclose(v, transform @ u)
    return transform