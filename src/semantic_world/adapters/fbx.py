import logging
import math
import os
from collections import deque
from dataclasses import dataclass
from typing import Optional, List


try:
    import bpy
    from bpy import types as bpy_types

    BlenderObject = bpy_types.Object
except ImportError:
    bpy = None
    BlenderObject = None
    logging.warning("bpy not found")

from ..connections import Connection6DoF, FixedConnection
from ..geometry import TriangleMesh
from ..prefixed_name import PrefixedName
from ..spatial_types.spatial_types import TransformationMatrix, Point3, RotationMatrix, Quaternion
from ..world import World, Body

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R

def blender_mesh_to_trimesh(obj: BlenderObject,
                            apply_modifiers: bool = True,
                            preserve_world_transform: bool = False) -> trimesh.Trimesh:
    """
    Convert a Blender mesh object to a trimesh.Trimesh.
    Ensures that all faces are triangles so NumPy and trimesh receive
    a homogeneous (N, 3) array.
    """
    if obj.type != 'MESH':
        raise TypeError(f"Object {obj.name} is not a mesh")

    # Obtain a (possibly evaluated) mesh datablock
    if apply_modifiers:
        depsgraph = bpy.context.evaluated_depsgraph_get()
        obj_eval = obj.evaluated_get(depsgraph)
        mesh = obj_eval.to_mesh()
    else:
        mesh = obj.to_mesh()

    try:
        # Vertex positions ----------------------------------------------------
        if preserve_world_transform:
            mat = obj.matrix_world
            verts = np.array([mat @ v.co for v in mesh.vertices], dtype=np.float64)
        else:
            verts = np.array([v.co[:] for v in mesh.vertices], dtype=np.float64)

        # Faces – always triangles -------------------------------------------
        mesh.calc_loop_triangles()                       # triangulate on-the-fly
        faces = np.array([t.vertices[:] for t in mesh.loop_triangles],
                         dtype=np.int64)                 # (N, 3)

        # Build trimesh -------------------------------------------------------
        tri_mesh = trimesh.Trimesh(vertices=verts,
                                   faces=faces,
                                   process=True,
                                   validate=True)
        return tri_mesh
    finally:
        # Free temporary mesh datablock created by to_mesh()
        if apply_modifiers:
            obj_eval.to_mesh_clear()
        else:
            obj.to_mesh_clear()


def get_min_z(root: BlenderObject) -> float:
    min_z = np.inf
    for x, y, z in root.bound_box:
        min_z = min(z, min_z)
    return min_z


@dataclass
class FBXParser:
    file_path: str
    """
    The path of the FBX file.
    """

    prefix: Optional[str] = None
    """
    The prefix for every name used in this world.
    """

    def __post_init__(self):
        if self.prefix is None:
            self.prefix = os.path.basename(self.file_path).split('.')[0]

    def parse(self) -> List[World]:
        """
        Parse the FBX file into a list of World objects.
        The fbx file is split at the top group level.
        """
        import mathutils

        # === Clean the scene ===
        bpy.ops.wm.read_factory_settings(use_empty=True)

        # import the fbx
        bpy.ops.import_scene.fbx(filepath=self.file_path, axis_forward='Y', axis_up='Z', global_scale=1.,
                                 bake_space_transform=True,
                                 # automatic_bone_orientation=False,  # keep bones untouched
                                 )

        # Clean up non-mesh objects
        for obj in list(bpy.context.scene.objects):
            if obj.type != 'MESH':
                bpy.data.objects.remove(obj, do_unlink=True)

        # Apply transform and reset origin
        for obj in bpy.context.scene.objects:
            if obj.type == 'MESH':
                bpy.context.view_layer.objects.active = obj
                bpy.ops.object.select_all(action='DESELECT')
                obj.select_set(True)
                bpy.ops.object.transform_apply(location=True, rotation=True, scale=True, isolate_users=True)
                bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
                obj.select_set(False)

        if bpy.context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        # Make mesh data single-user to avoid rotating shared meshes multiple times
        for obj in bpy.context.scene.objects:
            if obj.type == 'MESH' and obj.data.users > 1:
                obj.data = obj.data.copy()

        # Rotation that remaps +Y -> +X (axes rotation)
        R_axes = mathutils.Matrix.Rotation(-math.pi / 2.0, 4, 'Z')
        R_axes_inv = R_axes.inverted()

        def reaxis_to_x_forward(obj):
            """Rotate object's local axes so X is forward, preserve children world transforms."""
            # Cache direct children's world matrices (all types, to keep chains intact)
            child_world = {c: c.matrix_world.copy() for c in obj.children}

            # Rotate the object's local axes by post-multiplying its world matrix
            obj.matrix_world = obj.matrix_world @ R_axes

            # Counter-rotate mesh data so the object doesn't move visually
            if obj.type == 'MESH':
                obj.data.transform(R_axes_inv)
                obj.data.update()

            # Restore children's world transforms so they don't shift
            for c, Mw in child_world.items():
                c.matrix_world = Mw

            # Recurse so children also get their axes reset (and their own kids preserved)
            for c in obj.children:
                reaxis_to_x_forward(c)

        # Start from roots so recursion covers the whole tree exactly once
        roots = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH' and obj.parent is None]
        for root in roots:
            reaxis_to_x_forward(root)

        return [self.parse_single_world(obj) for obj in bpy.context.scene.objects
                if obj.type == 'MESH' and obj.parent is None]


    def parse_single_world(self, root: BlenderObject) -> World:
        """
        Create a world from a group in the FBX file.

        :param root: The root of the group.
        :return: The world
        """
        # move the group to the center of the world
        root.location = (0, 0, 0)

        # this is used to align the bottom of the group (for example the dresser) to the ground. But tbh I dont think
        # we want this, because thats what usually the footprint should be fore for:
        # map -> foot_print -> center of the group
        # min_z = get_min_z(root)
        # root.location = (0, 0, -min_z)

        # rotate the group to have the x-axis be the forward axis. this rotation was taken from
        # the dressers_group.fbx, it may not apply to all procthor fbx files
        rot_z = math.radians(90)
        root.rotation_euler.rotate_axis("Z", rot_z)

        # Apply the rotation to set +X as the new forward direction
        bpy.context.view_layer.objects.active = root
        bpy.ops.object.select_all(action='DESELECT')
        root.select_set(True)
        bpy.ops.object.transform_apply(rotation=True)
        root.select_set(False)

        # create the resulting world and make a footprint as root
        world = World(name=self.prefix)
        # base_foot_print = Body(name=PrefixedName(f"{self.prefix}_footprint", self.prefix))
        # world.add_body(base_foot_print)

        # initialize parent dict
        obj_to_body_map = {}

        # go through objects in bfs order
        object_queue = deque([root])
        with world.modify_world():
            while object_queue:
                obj = object_queue.popleft()

                name = PrefixedName(name=obj.name_full, prefix=self.prefix)

                # convert the mesh to trimesh
                mesh = blender_mesh_to_trimesh(obj)
                origin = np.array(obj.matrix_local)
                point = Point3.from_iterable(origin[:-1, 3])
                rotation = R.from_matrix(matrix=origin[:-1, :-1]).as_quat()
                quaterion = Quaternion.from_iterable(rotation)
                rotationmatrix = RotationMatrix.from_quaternion(quaterion)
                origin = TransformationMatrix.from_point_rotation_matrix(point, rotationmatrix)
                shape = TriangleMesh(mesh=mesh)

                # create the body
                body = Body(name=name, visual=[shape], collision=[shape])
                world.add_body(body)

                obj_to_body_map[obj] = body
                object_queue.extend(obj.children)

                # memoize this and create a connection
                if obj.parent is not None:
                    connection = FixedConnection(parent=obj_to_body_map[obj.parent],
                                                child=body,
                                                _world=world,
                                                origin_expression=origin)
                    world.add_connection(connection)

        return world
