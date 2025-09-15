from __future__ import absolute_import, annotations

from functools import lru_cache
from typing import Dict, Tuple, TYPE_CHECKING
from typing_extensions import OrderedDict

import numpy as np
import rustworkx.visit

from ..datastructures.prefixed_name import PrefixedName
from ..datastructures.types import NpMatrix4x4
from ..spatial_types import spatial_types as cas
from ..spatial_types.math import inverse_frame

from ..world_description.world_entity import Connection, KinematicStructureEntity

if TYPE_CHECKING:
    from ..world import World


class ForwardKinematicsVisitor(rustworkx.visit.DFSVisitor):
    """
    Visitor class for collection various forward kinematics expressions in a world model.

    This class is designed to traverse a world, compute the forward kinematics transformations in batches for different
    use cases.
    1. Efficient computation of forward kinematics between any bodies in the world.
    2. Efficient computation of forward kinematics for all bodies with collisions for updating collision checkers.
    3. Efficient computation of forward kinematics as position and quaternion, useful for ROS tf.
    """

    compiled_collision_fks: cas.CompiledFunction
    compiled_all_fks: cas.CompiledFunction

    forward_kinematics_for_all_bodies: np.ndarray
    """
    A 2D array containing the stacked forward kinematics expressions for all bodies in the world.
    Dimensions are ((number of bodies) * 4) x 4.
    They are computed in batch for efficiency.
    """
    body_name_to_forward_kinematics_idx: Dict[PrefixedName, int]
    """
    Given a body name, returns the index of the first row in `forward_kinematics_for_all_bodies` that corresponds to that body.
    """

    def __init__(self, world: World):
        self.world = world
        self.child_body_to_fk_expr: Dict[PrefixedName, cas.TransformationMatrix] = {
            self.world.root.name: cas.TransformationMatrix()
        }
        self.tf: Dict[Tuple[PrefixedName, PrefixedName], cas.Expression] = OrderedDict()

    def connection_call(self, edge: Tuple[int, int, Connection]):
        """
        Gathers forward kinematics expressions for a connection.
        """
        connection = edge[2]
        map_T_parent = self.child_body_to_fk_expr[connection.parent.name]
        self.child_body_to_fk_expr[connection.child.name] = map_T_parent.dot(
            connection.origin_expression
        )
        self.tf[(connection.parent.name, connection.child.name)] = (
            connection.origin_as_position_quaternion()
        )

    tree_edge = connection_call

    def compile_forward_kinematics(self) -> None:
        """
        Compiles forward kinematics expressions for fast evaluation.
        """
        all_fks = cas.vstack(
            [
                self.child_body_to_fk_expr[body.name]
                for body in self.world.kinematic_structure_entities
            ]
        )
        tf = cas.vstack([pose for pose in self.tf.values()])
        collision_fks = []
        for body in sorted(
            self.world.bodies_with_enabled_collision, key=lambda b: b.name
        ):
            if body == self.world.root:
                continue
            collision_fks.append(self.child_body_to_fk_expr[body.name])
        collision_fks = cas.vstack(collision_fks)
        params = [v.symbols.position for v in self.world.degrees_of_freedom]
        self.compiled_all_fks = all_fks.compile(parameters=params)
        self.compiled_collision_fks = collision_fks.compile(parameters=params)
        self.compiled_tf = tf.compile(parameters=params)
        self.idx_start = {
            body.name: i * 4
            for i, body in enumerate(self.world.kinematic_structure_entities)
        }

    def recompute(self) -> None:
        """
        Clears cache and recomputes all forward kinematics. Should be called after a state update.
        """
        self.compute_forward_kinematics_np.cache_clear()
        self.subs = self.world.state.positions
        self.forward_kinematics_for_all_bodies = self.compiled_all_fks.fast_call(
            self.subs
        )
        self.collision_fks = self.compiled_collision_fks.fast_call(self.subs)

    def compute_tf(self) -> np.ndarray:
        """
        Computes a (number of bodies) x 7 matrix of forward kinematics in position/quaternion format.
        The rows are ordered by body name.
        The first 3 entries are position values, the last 4 entires are quaternion values in x, y, z, w order.

        This is not updated in 'recompute', because this functionality is only used with ROS.
        :return: A large matrix with all forward kinematics.
        """
        return self.compiled_tf.fast_call(self.subs)

    @lru_cache(maxsize=None)
    def compute_forward_kinematics_np(
        self, root: KinematicStructureEntity, tip: KinematicStructureEntity
    ) -> NpMatrix4x4:
        """
        Computes the forward kinematics from the root body to the tip body, root_T_tip.

        This method computes the transformation matrix representing the pose of the
        tip body relative to the root body, expressed as a numpy ndarray.

        :param root: Root body for which the kinematics are computed.
        :param tip: Tip body to which the kinematics are computed.
        :return: Transformation matrix representing the relative pose of the tip body with respect to the root body.
        """
        root = root.name
        tip = tip.name
        root_is_world = root == self.world.root.name
        tip_is_world = tip == self.world.root.name

        if not tip_is_world:
            i = self.idx_start[tip]
            map_T_tip = self.forward_kinematics_for_all_bodies[i : i + 4]
            if root_is_world:
                return map_T_tip

        if not root_is_world:
            i = self.idx_start[root]
            map_T_root = self.forward_kinematics_for_all_bodies[i : i + 4]
            root_T_map = inverse_frame(map_T_root)
            if tip_is_world:
                return root_T_map

        if tip_is_world and root_is_world:
            return np.eye(4)

        return root_T_map @ map_T_tip
