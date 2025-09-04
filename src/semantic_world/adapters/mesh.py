import os
from dataclasses import dataclass, field
from enum import Enum

import fbxloader
import numpy as np
import trimesh
from fbxloader.nodes import Mesh as FBXMesh, Object3D, Scene

from ..spatial_types.spatial_types import RotationMatrix
from ..world_description.connections import FixedConnection
from ..world_description.geometry import FileMesh, TriangleMesh
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types import TransformationMatrix, Point3
from ..world import World
from ..world_description.world_entity import Body


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

        mesh_shape = FileMesh(origin=TransformationMatrix(), filename=self.file_path)
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
class CoordinateAxis(Enum):
    """
    Enum for coordinate axes with direction.
    The value is a tuple of (axis_index, sign), where axis_index is 0 for X, 1 for Y, and 2 for Z,
    and sign is 1 for positive direction and -1 for negative direction.
    """

    X = (0, 1)
    NEGATIVE_X = (0, -1)
    Y = (1, 1)
    NEGATIVE_Y = (1, -1)
    Z = (2, 1)
    NEGATIVE_Z = (2, -1)

    @classmethod
    def from_fbx(cls, axis_value: int, sign: int):
        """Map FBX axis index + sign into a CoordinateAxis enum."""
        for member in cls:
            if member.value[0] == axis_value and member.value[1] == sign:
                return member

    def to_vector(self):
        """Convert the CoordinateAxis to a 3D unit vector."""
        idx, sgn = self.value
        v = np.zeros(3)
        v[idx] = float(sgn)
        return v


@dataclass
class FBXGlobalSettings:
    """
    Class to handle FBX global settings, particularly the coordinate system.
    This class extracts the up, front, and coordinate axes from the FBX file and provides
    a method to get the transformation matrix from FBX to Semantic World coordinate system.
    """

    fbx_loader: fbxloader.FBXLoader
    """
    The FBXLoader instance containing the loaded FBX file.
    """

    up_axis: CoordinateAxis = field(init=False)
    """The up axis of the FBX file."""

    front_axis: CoordinateAxis = field(init=False)
    """The front axis of the FBX file."""

    coord_axis: CoordinateAxis = field(init=False)
    """The final, third axis of the FBX file, called the coordinate axis."""

    def __post_init__(self):
        fbx = self.fbx_loader

        self.up_axis = CoordinateAxis.from_fbx(
            fbx.fbxtree["GlobalSettings"]["UpAxis"]["value"],
            fbx.fbxtree["GlobalSettings"]["UpAxisSign"]["value"],
        )

        self.front_axis = CoordinateAxis.from_fbx(
            fbx.fbxtree["GlobalSettings"]["FrontAxis"]["value"],
            fbx.fbxtree["GlobalSettings"]["FrontAxisSign"]["value"],
        )

        self.coord_axis = CoordinateAxis.from_fbx(
            fbx.fbxtree["GlobalSettings"]["CoordAxis"]["value"],
            fbx.fbxtree["GlobalSettings"]["CoordAxisSign"]["value"],
        )

    def get_semantic_world_T_fbx(self):
        """
        Get the transformation matrix from FBX to Semantic World coordinate system.
        """
        sX = self.front_axis.to_vector()
        sY = self.coord_axis.to_vector()
        sZ = self.up_axis.to_vector()
        S = np.column_stack((sX, sY, sZ))

        R = S.T

        M = np.eye(4)
        M[:3, :3] = R
        return M


@dataclass
class FBXParser(MeshParser):
    """
    Adapter for FBX files.
    """

    @staticmethod
    def transform_vertices(
        vertices: np.ndarray, semantic_world_T_fbx: np.ndarray
    ) -> np.ndarray:
        """
        Transform vertices from FBX coordinate system to Semantic World coordinate system.
        """
        assert vertices.ndim == 2 and vertices.shape[1] == 3, "vertices must be (N,3)"
        assert semantic_world_T_fbx.shape == (4, 4), "semantic_world_T_fbx must be 4x4"

        ones = np.ones((vertices.shape[0], 1), dtype=vertices.dtype)
        vert_T_fbx = np.concatenate([vertices, ones], axis=1)

        vert_T_semantic_world = vert_T_fbx @ semantic_world_T_fbx.T
        return vert_T_semantic_world[:, :3]

    def parse(self) -> World:
        """
        Parse the FBX file, each object in the FBX file is converted to a body in the world and the meshes are loaded
        as TriangleMesh objects.

        :return: A World containing content of the FBX file.
        """
        fbx = fbxloader.FBXLoader(self.file_path)

        global_settings = FBXGlobalSettings(fbx)
        semantic_world_T_fbx = global_settings.get_semantic_world_T_fbx()

        world = World()

        with world.modify_world():
            for obj_id, obj in fbx.objects.items():
                # Create a body for each object in the FBX file
                if type(obj) is Object3D:
                    name = fbx.fbxtree["Objects"]["Model"][obj_id]["attrName"].split(
                        "\x00"
                    )[0]
                    meshes = []
                    for o in obj.children:
                        if isinstance(o, FBXMesh):
                            aligned_vertices = (
                                self.transform_vertices(
                                    o.vertices, semantic_world_T_fbx
                                )
                                / 100
                            )

                            t_mesh = TriangleMesh(
                                origin=TransformationMatrix(),
                                mesh=trimesh.Trimesh(
                                    vertices=aligned_vertices, faces=o.faces
                                ),
                            )

                            meshes.append(t_mesh)
                    body = Body(
                        name=PrefixedName(name), collision=meshes, visual=meshes
                    )
                    world.add_body(body)

            for obj in fbx.objects.values():
                if type(obj) is Object3D:
                    name = fbx.fbxtree["Objects"]["Model"][obj.id]["attrName"].split(
                        "\x00"
                    )[0]
                    parent_name = (
                        fbx.fbxtree["Objects"]["Model"][obj.parent.id][
                            "attrName"
                        ].split("\x00")[0]
                        if type(obj.parent) is not Scene
                        else None
                    )
                    if not parent_name:
                        continue

                    obj_body = world.get_body_by_name(name)
                    parent_body = world.get_body_by_name(parent_name)

                    parent_P_child = Point3(*obj.matrix[3, :3])
                    parent_R_child = RotationMatrix(obj.matrix)
                    parent_T_child = TransformationMatrix.from_point_rotation_matrix(
                        parent_P_child, parent_R_child, reference_frame=parent_body
                    )

                    connection = FixedConnection(
                        parent=parent_body,
                        child=obj_body,
                        _world=world,
                        origin_expression=parent_T_child,
                    )
                    world.add_connection(connection)

        return world
