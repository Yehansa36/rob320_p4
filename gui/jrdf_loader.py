import os
import json

import numpy as np
import open3d as o3d

import open3d.visualization.rendering

from scipy.spatial.transform import Rotation

HOME = os.path.expanduser("~")


def quaternion_to_rotation_matrix(q) -> np.ndarray:
    """
    Convert [w, x, y, z] quaternion to 3x3 rotation matrix.
    """
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def origin_to_matrix(origin):
    # input transform contains translation and rotation
    # translation contains x, y, and z
    # rotation is quaternion that contains w, x, y, z
    # Convert transform message into 4x4 homogenous transformation matrix
    t = np.eye(4)
    t[:3, 3] = origin["xyz"]
    t[:3, :3] = Rotation.from_euler("XYZ", origin["rpy"]).as_matrix()
    return t


class RobotModel:
    def __init__(self):
        self.visual_transforms = {}
        self.visuals = {}  # link name to entity mapping
        self.visual_origins = {}  # map of visual visual_origins

        self.collision_transforms = {}
        self.collisions = {}  # link name to entity mapping
        self.collision_origins = {}  # map of visual visual_origins

        self.original_colors = {}  # store original colors for restoring after hover

    def get_visuals(self):
        return self.visuals

    def get_collisions(self):
        return self.collisions

    def update_pose(self, global_tf: dict, vis) -> None:
        if global_tf is None:
            return

        for link_name, transform in global_tf.items():
            if link_name in self.visuals:
                for i, mesh in enumerate(self.visuals[link_name]):
                    # TODO: This could probably be vectorized ...
                    prev_t_inv = np.linalg.inv(self.visual_transforms[link_name][i])
                    t_visual = transform @ self.visual_origins[link_name][i]
                    t_new = t_visual @ prev_t_inv
                    mesh.transform(t_new)
                    self.visual_transforms[link_name][i] = t_visual

            if link_name in self.collisions:
                for i, mesh in enumerate(self.collisions[link_name]):
                    prev_t_inv = np.linalg.inv(self.collision_transforms[link_name][i])
                    t_collision = transform @ self.collision_origins[link_name][i]
                    t_new = t_collision @ prev_t_inv
                    mesh.transform(t_new)
                    self.collision_transforms[link_name][i] = t_collision


class JRDFLoader:
    def parse_geometry(self, geometry: dict):

        mesh = None
        if geometry["type"] == "box":
            size = geometry["size"]
            mesh = o3d.geometry.TriangleMesh.create_box(
                width=size[0], height=size[1], depth=size[2]
            )
            mesh.compute_vertex_normals()
        elif geometry["type"] == "cylinder":
            radius = geometry["radius"]
            length = geometry["length"]
            mesh = o3d.geometry.TriangleMesh.create_cylinder(
                radius=radius, height=length
            )
            mesh.compute_vertex_normals()
        elif geometry["type"] == "sphere":
            radius = geometry["radius"]
            mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            mesh.compute_vertex_normals()
        elif geometry["type"] == "mesh":
            filename = HOME + "/.rix/models/" + geometry["filename"]
            mesh = o3d.io.read_triangle_mesh(filename)
            if "scale" in geometry:
                scale = geometry["scale"]
                mesh.vertices = o3d.utility.Vector3dVector(
                    np.asarray(mesh.vertices) * np.array(scale)
                )
            mesh.compute_vertex_normals()
            mesh.compute_triangle_normals()
        else:
            raise ValueError(f"Unsupported geometry type: {geometry['type']}")

        return mesh

    def parse_link(self, link: dict, materials):
        link_name = link["name"]
        visuals = []
        visual_origins = []
        collisions = []
        collision_origins = []
        default_color = [70 / 255, 70 / 255, 70 / 255]
        if "visuals" in link:

            for visual in link["visuals"]:
                material = None
                if "material" in visual:
                    if visual["material"]["name"] in materials:
                        material = materials[visual["material"]["name"]]
                    else:
                        material = visual["material"]

                origin = np.eye(4)
                if "origin" in visual:
                    origin = origin_to_matrix(visual["origin"])
                visual_origins.append(origin)

                # Call the parse geometry function.
                mesh = self.parse_geometry(visual["geometry"])
                if material is not None:
                    if "color" in material:
                        mesh.paint_uniform_color(
                            material["color"][:3]
                        )  # Set color, optional
                    elif "filename" in material:
                        if np.asarray(mesh.triangle_uvs).size == 0:
                            mesh.paint_uniform_color(
                                default_color
                            )  # Set color, optional
                        else:
                            texture_path = HOME + "/.rix/models/" + material["filename"]
                            texture = o3d.io.read_image(texture_path)
                            mesh.textures = [o3d.geometry.Image(texture)]

                visuals.append(mesh)

        if "collisions" in link:
            for collision in link["collisions"]:
                origin = np.eye(4)
                if "origin" in collision:
                    origin = origin_to_matrix(collision["origin"])
                collision_origins.append(origin)

                # Parse the collision cobject
                mesh = self.parse_geometry(collision["geometry"])

                collisions.append(mesh)

        return link_name, visuals, visual_origins, collisions, collision_origins

    def load(self, model_path: str, vis) -> RobotModel:
        robot = RobotModel()
        with open(model_path, "r") as file:
            data = json.load(file)
            for link in data["links"]:
                materials = {}
                if "materials" in data:
                    for material in data["materials"]:
                        materials[material["name"]] = material
                link_name, visuals, visual_origins, collisions, collision_origins = (
                    self.parse_link(link, materials)
                )
                # vis.add_geometries(visuals)
                for e in visuals:
                    vis.add_geometry(e)

                robot.visuals[link_name] = visuals
                robot.visual_transforms[link_name] = [
                    np.eye(4) for i in range(len(visuals))
                ]  # Initialize an array of size len(visuals) with identity matrices
                robot.visual_origins[link_name] = visual_origins

                robot.collisions[link_name] = collisions
                robot.collision_transforms[link_name] = [
                    np.eye(4) for i in range(len(collisions))
                ]  # Initialize an array of size len(collisions) with identity matrices
                robot.collision_origins[link_name] = collision_origins
        return robot


# TODO: Material properties not working...why?
