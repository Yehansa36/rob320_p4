import os
import math
import numpy as np
import open3d as o3d

HOME = os.path.expanduser("~")

class Scene:
    def __init__(self, 
                 vis,
                 plane_color=np.array([130/255,155/255,200/255]), 
                 desk_color=np.array([222/255,184/255,135/255]),
                 wall_color=np.array([0/255,39/255,76/255]),
                 mesh_text_color=np.array([255/255,203/255,5/255]),
                 grid_color=np.array([0.0, 0.0, 0.0]),
                 plane_width=10.0,
                 plane_depth=0.01,
                 plane_density=2.0,
                 desk_width=4.0,
                 desk_height=2.0,
                 desk_depth=0.05,
                 wall_depth=0.5
                 ):
        vis.get_render_option().background_color = np.array([193/255,217/255,245/255])
        vis.get_render_option().show_coordinate_frame = True
        vis.get_render_option().light_on = True

        filename = "block_m.obj"    # https://www.stlfinder.com/model/university-of-michigan-block-m-b7sInsVu/2530960
        self.block_m_mesh = o3d.io.read_triangle_mesh(filename)

        if self.block_m_mesh.is_empty():
            raise ValueError(f"Mesh is empty — failed to load STL at {filename}.")

        self.block_m_mesh.compute_vertex_normals()
        self.block_m_mesh.compute_triangle_normals()

        self.block_m_mesh.scale(scale = 0.01, center = np.array([0, 0, 0]))
        self.block_m_mesh.translate(np.array([-desk_width / 4, desk_height / 1.5, 0.]))
        # self.block_m_mesh.translate(np.array([-desk_width / 2, -desk_height / 2, desk_depth]))
        # block_rotate = o3d.geometry.TriangleMesh.get_rotation_matrix_from_xyz(np.array([0., 0., np.pi / 14]))
        # self.block_m_mesh.rotate(block_rotate)
        self.block_m_mesh.paint_uniform_color(mesh_text_color)

        self.block_m_mesh.get_oriented_bounding_box()

        vis.add_geometry(self.block_m_mesh)

        filename = "lfp_logo.obj"
        self.lfp_logo_mesh = o3d.io.read_triangle_mesh(filename)

        if self.lfp_logo_mesh.is_empty():
            raise ValueError(f"Mesh is empty — failed to load STL at {filename}.")

        self.lfp_logo_mesh.compute_vertex_normals()
        self.lfp_logo_mesh.compute_triangle_normals()

        self.lfp_logo_mesh.scale(scale = 2.5, center = np.array([0, 0, 0]))
        logo_rotate = o3d.geometry.TriangleMesh.get_rotation_matrix_from_xyz(np.array([np.pi / 2, 0, np.pi / 3]))
        self.lfp_logo_mesh.rotate(logo_rotate)
        self.lfp_logo_mesh.translate(np.array([desk_width / 3, desk_height / 1.75, .15]))
        self.lfp_logo_mesh.paint_uniform_color(mesh_text_color)
        vis.add_geometry(self.lfp_logo_mesh)

        desk = o3d.geometry.TriangleMesh.create_box(width = desk_width, height = desk_height, depth = desk_depth)
        desk.translate(np.array([-desk_width / 2,-desk_height / 2, -desk_depth]))
        desk.paint_uniform_color(desk_color)
        vis.add_geometry(desk)

        wall = o3d.geometry.TriangleMesh.create_box(width = desk_width, height = desk_depth, depth = wall_depth)
        wall.translate(np.array([-desk_width / 2, desk_height / 2, 0]))
        wall.paint_uniform_color(wall_color)
        vis.add_geometry(wall)

        num_grid_lines = round(plane_width * plane_density)
        plane = o3d.geometry.TriangleMesh.create_box(width = plane_width, height = plane_width, depth = plane_depth / 2)
        plane.translate(np.array([-plane_width / 2., -plane_width / 2., -plane_depth * 4]))
        plane.paint_uniform_color(plane_color)
        vis.add_geometry(plane)

        plane.compute_triangle_normals()

        for i in range(num_grid_lines):
            grid_line1 = o3d.geometry.TriangleMesh.create_box(width = plane_width, height = plane_depth, depth = plane_depth)
            grid_line1.translate(np.array([-plane_width / 2, i / plane_density - plane_width / 2, -plane_depth * 4]))
            grid_line1.paint_uniform_color(grid_color)
            vis.add_geometry(grid_line1)
            grid_line2 = o3d.geometry.TriangleMesh.create_box(width = plane_depth, height = plane_width, depth = plane_depth)
            grid_line2.translate(np.array([i / plane_density - plane_width / 2, -plane_width / 2, -plane_depth * 4]))
            grid_line2.paint_uniform_color(grid_color)
            vis.add_geometry(grid_line2)

        