import os
from dataclasses import dataclass

import trimesh

from ..world_description import Connection6DoF
from ..spatial_types.spatial_types import RotationMatrix
from ..world_description import Mesh, TriangleMesh
from ..datastructures import PrefixedName
from ..spatial_types import TransformationMatrix, Point3
from ..world import World
from ..world_description import Body


@dataclass
class MeshParser:
    """
    Adapter for mesh files.
    """

    file_path: str
    """
    The path to the mesh file.
    """

    def parse(self) -> World:
        """
        Parse the mesh file to a body and return a world containing that body.

        :return: A World object containing the parsed body.
        """
        file_name = os.path.basename(self.file_path)

        mesh_shape = Mesh(origin=TransformationMatrix(), filename=self.file_path)
        body = Body(
            name=PrefixedName(file_name), collision=[mesh_shape], visual=[mesh_shape]
        )

        world = World()
        with world.modify_world():
            world.add_kinematic_structure_entity(body)

        return world


@dataclass
class STLParser(MeshParser):
    pass


@dataclass
class OBJParser(MeshParser):
    pass


@dataclass
class DAEParser(MeshParser):
    pass


@dataclass
class PLYParser(MeshParser):
    pass


@dataclass
class OFFParser(MeshParser):
    pass


@dataclass
class GLBParser(MeshParser):
    pass


@dataclass
class XYZParser(MeshParser):
    pass


@dataclass
class FBXParser(MeshParser):
    """
    Adapter for FBX files.
    """

    def parse(self) -> World:
        """
        Parse the FBX file, each object in the FBX file is converted to a body in the world and the meshes are loaded
        as TriangleMesh objects.

        :return: A World containing content of the FBX file.
        """
        import fbxloader
        from fbxloader.nodes import Mesh as FBXMesh, Object3D, Scene

        fbx = fbxloader.FBXLoader(self.file_path)
        world = World()

        with world.modify_world():
            for obj_id, obj in fbx.objects.items():
                # Create a body for each object in the FBX file
                if type(obj) is Object3D:
                    name = fbx.fbxtree["Objects"]["Model"][obj_id]["attrName"]
                    meshes = []
                    for o in obj.children:
                        if isinstance(o, FBXMesh):
                            t_mesh = TriangleMesh(
                                origin=TransformationMatrix(),
                                mesh=trimesh.Trimesh(
                                    vertices=o.vertices, faces=o.faces
                                ),
                            )
                            t_mesh.mesh.vertices = (
                                o.vertices[:, [0, 2, 1]] / 100
                            )  # Convert from cm to m and switch Y and Z axes
                            meshes.append(t_mesh)
                    body = Body(
                        name=PrefixedName(name), collision=meshes, visual=meshes
                    )
                    world.add_kinematic_structure_entity(body)

            for obj in fbx.objects.values():
                if type(obj) is Object3D:
                    name = fbx.fbxtree["Objects"]["Model"][obj.id]["attrName"]
                    parent_name = (
                        fbx.fbxtree["Objects"]["Model"][obj.parent.id]["attrName"]
                        if type(obj.parent) is not Scene
                        else None
                    )
                    if not parent_name:
                        continue

                    obj_body = world.get_kinematic_structure_entity_by_name(name)
                    parent_body = world.get_kinematic_structure_entity_by_name(
                        parent_name
                    )

                    translation = Point3(*obj.matrix[3, :3])
                    rotation_matrix = RotationMatrix(obj.matrix)
                    parent_T_child = TransformationMatrix.from_point_rotation_matrix(
                        translation, rotation_matrix, reference_frame=parent_body
                    )

                    connection = Connection6DoF(
                        parent=parent_body,
                        child=obj_body,
                        _world=world,
                        origin_expression=parent_T_child,
                    )
                    world.add_connection(connection)

        return world
