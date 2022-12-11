from utils import get_rotating_transform as gR
import time

import open3d as o3d
import numpy as np
import geomstats as gm
from geomstats.geometry.special_orthogonal import SpecialOrthogonal


from utils import (create_sphere_pcl, create_sphere_mesh,
                   setup_canvas, get_axis_angle_from_matrix,
                   gen_spherical_gaussian_axis)

# Setup canvas
z_init = 1000
s = 500.
plane_height = 480*2
plane_width = 640*2

# Create tangent plane
# plane_height = 255
# plane_width = 255
plane_pcd = setup_canvas(width=plane_width, height=plane_height)
plane_pcd.translate((0, 0, s))
tangent_coord = o3d.geometry.TriangleMesh(
).create_coordinate_frame(size=100, origin=(0, 0, s))

# coords = o3d.geometry.create_mesh_coordinate_frame(size=1.0, origin=(0,0,0))
# coords = o3d.geometry.TriangleMesh.create_mesh_coordinate_frame(size=1.0, origin=(0,0,0))
coords = o3d.geometry.TriangleMesh().create_coordinate_frame(
    size=40, origin=(0, 0, 0))

# Create an instance of the SpecialOrthogonal group SO(3)
SO3 = SpecialOrthogonal(3)

rot_point = o3d.geometry.PointCloud()
rot_point.points = o3d.utility.Vector3dVector([[0., 0., 1000.]])
rot_axis = o3d.geometry.TriangleMesh().create_arrow(
    cylinder_radius=10, cylinder_height=200.,
    cone_radius=10., cone_height=20.)
mat_rot_axis = o3d.visualization.rendering.MaterialRecord()
mat_rot_axis.base_color = [1., 0., 0., 0.5]
rot_axis.translate((0, 0, s))

# Add mesh

# sphere_mesh, mat_sphere = create_sphere_mesh(coords=(0,0,z_init), scale=s)
radius = s
sphere_mesh, mat_sphere = create_sphere_mesh(coords=(0, 0, 0), radius=radius)

mesh = o3d.io.read_triangle_mesh("models/obj_000001.ply")
# Scale and center
mesh.vertices = o3d.utility.Vector3dVector(
    (np.asarray(mesh.vertices) - np.asarray(mesh.vertices).mean(axis=0))*1000)

R = SO3.random_uniform()
axis, angle = get_axis_angle_from_matrix(R)
print(f"axis: {axis}, angle: {angle}")
# Animate
# Camera intrinsic matrix for a camera at plane_pcd
cam_width = plane_width //2
cam_height = plane_height//2
fx = cam_width/2
fy = cam_height/2
cx = cam_width/2; cy = cam_height/2
# cx = 0; cy = 0
K = np.array([  [fx, 0, cx],
                [0, fy, cy],
                [0,  0,  1]])


def make_line(axis):
    line = np.array([t*axis for t in np.linspace(-s, s, 1000)])
    line_pcl = o3d.geometry.PointCloud()
    line_pcl.points = o3d.utility.Vector3dVector(line)

    return line_pcl


def animate_projection(vis, K=K, mesh=mesh):

    # Sample rotation
    # global SO3
    # R = SO3.random_uniform()
    # axis, angle = get_axis_angle_from_matrix(R)

    # axis = np.random.random(3)
    global axis
    axis = gen_spherical_gaussian_axis(old_axis=axis)
    axis = axis/np.linalg.norm(axis)
    angle = np.random.random(1)*2*np.pi/100
    # R_ = gR(u=rot_point.points/np.linalg.norm(rot_point.points), v=axis/np.linalg.norm(axis))
    print(f"axis: {axis}, angle: {angle}")

    # Rotate object
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation=angle*axis)

    mesh.rotate(R)
    vis.remove_geometry('object')
    vis.add_geometry(name='object', geometry=mesh)

    # Transport axis of rotation
    # Transform the mesh using the projection matrix
    # update_axis(vis=vis, axis=axis)
    vis.remove_geometry('rot_axis')
    rot_axis = make_line(axis)
    vis.add_geometry(name='rot_axis', geometry=rot_axis)
    # rot_axis.verts = o3d.utility.Vector3dVector(SO3.right_translate_by_rotation(rot_axis.vertices, R))

    # Project points to the camera at tangent plane plane_pcd and
    # visualize them as a white points on the plane.
    plane_colors = np.zeros((plane_width, plane_height, 3))
    plane_pts = np.dot(K, np.asarray(mesh.vertices).T).T
    # plane_pts = plane_pts//plane_pts[:, 2, np.newaxis]
    plane_pts = plane_pts//plane_pts[:, 2, None]
    inlier_mask = np.zeros(plane_pts.shape[0], dtype=bool)
    inlier_mask[
        (plane_pts[:, 0] > 0)*(plane_pts[:, 0] < cam_width)*(plane_pts[:, 1] > 0)*(plane_pts[:, 1] < cam_height)
        ] = True
    # plane_pts[plane_pts[:, 0] < 0] = 0
    # plane_pts[plane_pts[:, 1] < 0] = 0
    # plane_pts[plane_pts[:, 0] >= width] = 0
    # plane_pts[plane_pts[:, 1] >= height] = 0
    plane_colors[plane_pts[inlier_mask][:, 0].astype(int), plane_pts[inlier_mask][:, 1].astype(int)] = [
        1, 1, 1]
    # plane_pcd.points = o3d.utility.Vector3dVector(
    # np.asarray(plane_pcd.points)[plane_pts[:,0].astype(int), plane_pts[:,1].astype(int), :])
    plane_pcd.colors = o3d.utility.Vector3dVector(plane_colors.reshape(-1, 3, order='C'))
    vis.remove_geometry('plane')
    vis.add_geometry(name='plane', geometry=plane_pcd)

    # P = np.asarray(mesh.vertices)
    # P = P//P[2,:]
    # P[P[:,0]<0 or P[:,1] < 0] = 0
    # P[P[0,:]>width] = 0
    # P[P[1,:]>height] = 0
    # plane_pcd.points = o3d.utility.Vector3dVector(
    #     plane_pcd.vertices[P[0,:].astype(int), P[1,:].astype(int), :])
    # vis.remove_geometry('plane')
    # vis.add_geometry(name='plane', geometry=plane_pcd)

    vis.remove_geometry('plane')
    vis.add_geometry(name='plane', geometry=plane_pcd)
    
    time.sleep(0.1)

    return True


# Create a visualizer and set the background color to white
vis = o3d.visualization.Visualizer()
# vis.get_render_option().background_color = np.ones(4)

# Add the point cloud and the mesh to the visualizer
geoms = [
    {'name': 'coords', 'geometry': coords},
    # {'name': 'tangent_coords', 'geometry': tangent_coord},
    {'name': 'rot_axis', 'geometry': rot_axis, 'material': mat_rot_axis},
    {'name': 'plane', 'geometry': plane_pcd},
    {'name': 'object', 'geometry': mesh},
    {'name': 'sphere', 'geometry': sphere_mesh, 'material': mat_sphere},
    # {'name': 'coords', 'geometry': coords, 'time': time},
    # # {'name': 'tangent_coords', 'geometry': tangent_coord},
    # {'name': 'rot_axis', 'geometry': rot_axis, 'material': mat_rot_axis, 'time': time},
    # {'name': 'plane', 'geometry': plane_pcd, 'time': time},
    # {'name': 'object', 'geometry': mesh, 'time': time},
    # {'name': 'sphere', 'geometry': sphere_mesh, 'material': mat_sphere, 'time': time},
]

from o3d_utils import draw
# window_uid = o3d.visualization.draw(geoms, title='projecion',
window_uid = draw(geoms, title='projecion',
                       actions=[('callback', animate_projection)],
                    #    animation_duration=100, animation_time_step=1.,
                       on_animation_tick=animate_projection,
                    #    on_animation_frame=animate_projection,
                    #    non_blocking_and_return_uid=True,
                       )

print("Window uid: ", window_uid, type(window_uid))
# o3d.visualization.gui.Application.instance.run()
# image = o3d.visualization.gui.Application.instance.render_to_image()
print(image.shape)
# window.scene.set_on_key(animate_projection, 'space')
