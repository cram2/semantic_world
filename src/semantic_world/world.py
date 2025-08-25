from __future__ import absolute_import
from __future__ import annotations

import logging
from copy import deepcopy
from dataclasses import dataclass, field
from enum import IntEnum
from functools import wraps, lru_cache
from typing import Dict, Tuple, OrderedDict, Union, Optional, Generic, TypeVar

import matplotlib.pyplot as plt
import numpy as np
import rustworkx as rx
import rustworkx.visit
import rustworkx.visualization
from typing_extensions import List, Type

from .connections import (
    HasUpdateState,
    Has1DOFState,
    Connection6DoF,
)
from .degree_of_freedom import DegreeOfFreedom
from .exceptions import (
    DuplicateViewError,
    AddingAnExistingViewError,
    ViewNotFoundError,
    AlreadyBelongsToAWorldError,
)
from .ik_solver import InverseKinematicsSolver
from .prefixed_name import PrefixedName
from .spatial_types import spatial_types as cas
from .spatial_types.derivatives import Derivatives
from .spatial_types.math import inverse_frame
from .types import NpMatrix4x4
from .utils import IDGenerator, copy_lru_cache
from .world_entity import Body, Connection, View
from .world_state import WorldState

logger = logging.getLogger(__name__)

id_generator = IDGenerator()

T = TypeVar("T")


class PlotAlignment(IntEnum):
    HORIZONTAL = 0
    VERTICAL = 1


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
            self.world.root.name: cas.TransformationMatrix()}
        self.tf: Dict[Tuple[PrefixedName, PrefixedName], cas.Expression] = OrderedDict()

    def connection_call(self, edge: Tuple[int, int, Connection]):
        """
        Gathers forward kinematics expressions for a connection.
        """
        connection = edge[2]
        map_T_parent = self.child_body_to_fk_expr[connection.parent.name]
        self.child_body_to_fk_expr[connection.child.name] = map_T_parent.dot(connection.origin_expression)
        self.tf[(connection.parent.name, connection.child.name)] = connection.origin_as_position_quaternion()

    tree_edge = connection_call

    def compile_forward_kinematics(self) -> None:
        """
        Compiles forward kinematics expressions for fast evaluation.
        """
        all_fks = cas.vstack([self.child_body_to_fk_expr[body.name] for body in self.world.bodies])
        tf = cas.vstack([pose for pose in self.tf.values()])
        collision_fks = []
        for body in self.world.bodies_with_collisions:
            if body == self.world.root:
                continue
            collision_fks.append(self.child_body_to_fk_expr[body.name])
        collision_fks = cas.vstack(collision_fks)
        params = [v.symbols.position for v in self.world.degrees_of_freedom]
        self.compiled_all_fks = all_fks.compile(parameters=params)
        self.compiled_collision_fks = collision_fks.compile(parameters=params)
        self.compiled_tf = tf.compile(parameters=params)
        self.idx_start = {body.name: i * 4 for i, body in enumerate(self.world.bodies)}

    def recompute(self) -> None:
        """
        Clears cache and recomputes all forward kinematics. Should be called after a state update.
        """
        self.compute_forward_kinematics_np.cache_clear()
        self.subs = self.world.state.positions
        self.forward_kinematics_for_all_bodies = self.compiled_all_fks.fast_call(self.subs)

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
    def compute_forward_kinematics_np(self, root: Body, tip: Body) -> NpMatrix4x4:
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
            map_T_tip = self.forward_kinematics_for_all_bodies[i:i + 4]
            if root_is_world:
                return map_T_tip

        if not root_is_world:
            i = self.idx_start[root]
            map_T_root = self.forward_kinematics_for_all_bodies[i:i + 4]
            root_T_map = inverse_frame(map_T_root)
            if tip_is_world:
                return root_T_map

        if tip_is_world and root_is_world:
            return np.eye(4)

        return root_T_map @ map_T_tip


class ResetStateContextManager:
    """
    A context manager for resetting the state of a given `World` instance.

    This class is designed to allow operations to be performed on a `World`
    object, ensuring that its state can be safely returned to its previous
    condition upon leaving the context. If no exceptions occur within the
    context, the original state of the `World` instance is restored, and the
    state change is notified.
    """

    def __init__(self, world: World):
        self.world = world

    def __enter__(self) -> None:
        self.state = deepcopy(self.world.state)

    def __exit__(self, exc_type: Optional[type], exc_val: Optional[Exception], exc_tb: Optional[type]) -> None:
        if exc_type is None:
            self.world.state = self.state
            self.world.notify_state_change()


class WorldModelUpdateContextManager:
    """
    Context manager for updating the state of a given `World` instance.
    This class manages that updates to the world within the context of this class only trigger recomputations after all
    desired updates have been performed.
    """
    first: bool = True

    def __init__(self, world: World):
        self.world = world

    def __enter__(self):
        if self.world.world_is_being_modified:
            self.first = False
        self.world.world_is_being_modified = True
        return self.world

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.first:
            self.world.world_is_being_modified = False
            if exc_type is None:
                self.world._notify_model_change()


def modifies_world(func):
    """
    Decorator that marks a method as a modification to the state or model of a world.
    """

    @wraps(func)
    def wrapper(self: World, *args, **kwargs):
        with self.modify_world() as context_manager:
            result = func(self, *args, **kwargs)
            return result

    return wrapper


@dataclass
class World:
    """
    A class representing the world.
    The world manages a set of bodies and connections represented as a tree-like graph.
    The nodes represent bodies in the world, and the edges represent joins between them.
    """

    kinematic_structure: rx.PyDAG[Body] = field(default_factory=lambda: rx.PyDAG(multigraph=False), kw_only=True,
                                                repr=False)
    """
    The kinematic structure of the world.
    The kinematic structure is a tree shaped directed graph where the nodes represent bodies in the world,
    and the edges represent connections between them.
    """

    views: List[View] = field(default_factory=list, repr=False)
    """
    All views the world is aware of.
    """

    degrees_of_freedom: List[DegreeOfFreedom] = field(default_factory=list)

    state: WorldState = field(default_factory=WorldState)
    """
    2d array where rows are derivatives and columns are dof values for that derivative.
    """

    _model_version: int = 0
    """
    The version of the model. This increases whenever a change to the kinematic model is made. Mostly triggered
    by adding/removing bodies and connections.
    """

    _state_version: int = 0
    """
    The version of the state. This increases whenever a change to the state of the kinematic model is made. 
    Mostly triggered by updating connection values.
    """

    world_is_being_modified: bool = False
    """
    Is set to True, when a function with @modifies_world is called or world.modify_world context is used.
    """

    name: Optional[str] = None
    """
    Name of the world. May act as default namespace for all bodies and views in the world which do not have a prefix.
    """

    @property
    def root(self) -> Body:
        """
        The root of the world is the unique node with in-degree 0.

        :return: The root of the world.
        """
        possible_roots = [node for node in self.bodies if self.kinematic_structure.in_degree(node.index) == 0]
        if len(possible_roots) == 1:
            return possible_roots[0]
        elif len(possible_roots) > 1:
            raise ValueError(f"More than one root found. Possible roots are {possible_roots}")
        else:
            raise ValueError(f"No root found.")

    def __hash__(self):
        return hash(id(self))

    def validate(self) -> bool:
        """
        Validate the world.

        The world must be a tree.
        :return: True if the world is valid, raises an AssertionError otherwise.
        """
        assert len(self.bodies) == (len(self.connections) + 1)
        assert rx.is_weakly_connected(self.kinematic_structure)
        actual_dofs = set()
        for connection in self.connections:
            actual_dofs.update(connection.dofs)
        assert actual_dofs == set(
            self.degrees_of_freedom
        ), "self.degrees_of_freedom does not match the actual dofs used in connections. Did you forget to call deleted_orphaned_dof()?"
        return True

    @modifies_world
    def add_degree_of_freedom(self, dof: DegreeOfFreedom) -> None:
        """
        Adds degree of freedom in the world.
        This is used to register DoFs that are not created by the world, but are part of the world model.
        :param dof: The degree of freedom to register.
        """
        if dof._world is self and dof in self.degrees_of_freedom:
            return
        if dof._world is not None:
            raise AlreadyBelongsToAWorldError("Cannot add a degree of freedom that already belongs to another world.")

        dof._world = self

        initial_position = 0
        lower_limit = dof.lower_limits.position
        if lower_limit is not None:
            initial_position = max(lower_limit, initial_position)
        upper_limit = dof.upper_limits.position
        if upper_limit is not None:
            initial_position = min(upper_limit, initial_position)
        self.state[dof.name].position = initial_position
        self.degrees_of_freedom.append(dof)

    def modify_world(self) -> WorldModelUpdateContextManager:
        return WorldModelUpdateContextManager(self)

    def reset_state_context(self) -> ResetStateContextManager:
        return ResetStateContextManager(self)

    def reset_cache(self) -> None:
        # super().reset_cache()
        # self.get_directly_controlled_child_links_with_collisions.cache_clear()
        # self.get_directly_controlled_child_links_with_collisions.cache_clear()
        # self.compute_chain_reduced_to_controlled_joints.cache_clear()
        # self.get_movable_parent_joint.cache_clear()
        # self.get_controlled_parent_joint_of_link.cache_clear()
        # self.get_controlled_parent_joint_of_joint.cache_clear()
        self.compute_split_chain_of_bodies.cache_clear()
        self.compute_split_chain_of_connections.cache_clear()
        # self.are_linked.cache_clear()
        # self.compose_fk_expression.cache_clear()
        self.compute_chain_of_bodies.cache_clear()
        self.compute_chain_of_connections.cache_clear()
        # self.is_link_controlled.cache_clear()
        for dof in self.degrees_of_freedom:
            dof.reset_cache()

    def notify_state_change(self) -> None:
        """
        If you have changed the state of the world, call this function to trigger necessary events and increase
        the state version.
        """
        # self.compute_fk.cache_clear()
        # self.compute_fk_with_collision_offset_np.cache_clear()
        self._recompute_forward_kinematics()
        self._state_version += 1

    def _notify_model_change(self) -> None:
        """
        Call this function if you have changed the model of the world to trigger necessary events and increase
        the model version number.
        """
        if not self.world_is_being_modified:
            self.reset_cache()
            self.compile_forward_kinematics_expressions()
            self.notify_state_change()
            self._model_version += 1
            self.validate()

    def delete_orphaned_dofs(self):
        actual_dofs = set()
        for connection in self.connections:
            actual_dofs.update(connection.dofs)
        self.degrees_of_freedom = list(actual_dofs)

    @property
    def bodies(self) -> List[Body]:
        """
        :return: A list of all bodies in the world.
        """
        return list(self.kinematic_structure.nodes())

    @property
    def bodies_with_collisions(self) -> List[Body]:
        """
        :return: A list of all bodies in the world that have collisions.
        """
        return [body for body in self.bodies if body.has_collision()]

    @property
    def connections(self) -> List[Connection]:
        """
        :return: A list of all connections in the world.
        """
        return list(self.kinematic_structure.edges())

    @modifies_world
    def add_body(self, body: Body) -> None:
        """
        Add a body to the world.

        :param body: The body to add.
        """
        if body._world is self and body.index is not None:
            return
        elif body._world is not None and body._world is not self:
            raise AlreadyBelongsToAWorldError("Cannot add a body that already belongs to another world.")

        body.index = self.kinematic_structure.add_node(body)

        # write self as the bodys world
        body._world = self

    @modifies_world
    def add_connection(self, connection: Connection) -> None:
        """
        Add a connection and the bodies it connects to the world.

        :param connection: The connection to add.
        """
        self.add_body(connection.parent)
        self.add_body(connection.child)
        connection._world = self
        self.kinematic_structure.add_edge(connection.parent.index, connection.child.index, connection)

    def add_view(self, view: View, exists_ok: bool = False) -> None:
        """
        Adds a view to the current list of views if it doesn't already exist. Ensures
        that the `view` is associated with the current instance and maintains the
        integrity of unique view names.

        :param view: The view instance to be added. Its name must be unique within
            the current context.
        :param exists_ok: Whether to raise an error or not when a view already exists.

        :raises AddingAnExistingViewError: If exists_ok is False and a view with the same name and type already exists.
        """
        try:
            self.get_view_by_name(view.name)
            if not exists_ok:
                raise AddingAnExistingViewError(view)
        except ViewNotFoundError:
            view._world = self
            self.views.append(view)

    def remove_view(self, view: View) -> None:
        """
        Removes a view from the current list of views if it exists.

        :param view: The view instance to be removed.
        """
        try:
            existing_view = self.get_view_by_name(view.name)
            if existing_view == view:
                self.views.remove(existing_view)
                view._world = None
            else:
                raise ValueError("The provided view instance does not match the existing view with the same name.")
        except ViewNotFoundError:
            logger.debug(f"View {view.name} not found in the world. No action taken.")

    def get_view_by_name(self, name: Union[str, PrefixedName]) -> Optional[View]:
        """
        Retrieves a View from the list of view based on its name.
        If the input is of type `PrefixedName`, it checks whether the prefix is specified and looks for an
        exact match. Otherwise, it matches based on the name's string representation.
        If more than one body with the same name is found, an assertion error is raised.
        If no matching body is found, a `ValueError` is raised.

        :param name: The name of the view to search for. Can be a string or a `PrefixedName` object.
        :return: The `View` object that matches the given name.
        """
        if isinstance(name, PrefixedName):
            if name.prefix is not None:
                matches = [view for view in self.views if view.name == name]
            else:
                matches = [view for view in self.views if view.name.name == name.name]
        else:
            matches = [view for view in self.views if view.name.name == name]
        if len(matches) > 1:
            raise DuplicateViewError(matches)
        if matches:
            return matches[0]
        raise ViewNotFoundError(name)

    def get_views_by_type(self, view_type: Type[Generic[T]]) -> List[T]:
        """
        Retrieves all views of a specific type from the world.

        :param view_type: The class (type) of the views to search for.
        :return: A list of `View` objects that match the given type.
        """
        return [view for view in self.views if isinstance(view, view_type)]

    @modifies_world
    def remove_body(self, body: Body) -> None:
        if body._world is self and body.index is not None:
            self.kinematic_structure.remove_node(body.index)
            body._world = None
            body.index = None
        else:
            logger.debug("Trying to remove a body that is not part of this world.")

    @modifies_world
    def remove_connection(self, connection: Connection) -> None:
        """
        Removes a connection and deletes the corresponding degree of freedom, if it was only used by this connection.
        Might create disconnected bodies, so make sure to add a new connection or delete the child body.

        :param connection: The connection to be removed
        """
        remaining_dofs = set()
        for remaining_connection in self.connections:
            if remaining_connection == connection:
                continue
            remaining_dofs.update(remaining_connection.dofs)

        for dof in connection.dofs:
            if dof not in remaining_dofs:
                self.degrees_of_freedom.remove(dof)
                del self.state[dof.name]
        self.kinematic_structure.remove_edge(connection.parent.index, connection.child.index)

    @modifies_world
    def merge_world(self, other: World, root_connection: Connection = None) -> None:
        """
        Merge a world into the existing one by merging degrees of freedom, states, connections, and bodies.
        This removes all bodies and connections from `other`.

        :param other: The world to be added.
        :param root_connection: If provided, this connection will be used to connect the two worlds. Otherwise, a new Connection6DoF will be created
        :return: None
        """

        self_root = self.root
        other_root = other.root
        for dof in other.degrees_of_freedom:
            self.state[dof.name].position = other.state[dof.name].position
            self.state[dof.name].velocity = other.state[dof.name].velocity
            self.state[dof.name].acceleration = other.state[dof.name].acceleration
            self.state[dof.name].jerk = other.state[dof.name].jerk
            dof._world = self
        self.degrees_of_freedom.extend(other.degrees_of_freedom)

        # do not trigger computations in other
        other.world_is_being_modified = True
        for connection in other.connections:
            other.remove_body(connection.parent)
            other.remove_body(connection.child)
            self.add_connection(connection)
        for body in other.bodies:
            if body._world is not None:
                other.remove_body(body)

        other_views = [view for view in other.views]
        for view in other_views:
            other.remove_view(view)
            self.add_view(view)

        other.world_is_being_modified = False

        connection = root_connection or Connection6DoF(
            parent=self_root, child=other_root, _world=self
        )
        for dof in connection.dofs:
            self.add_degree_of_freedom(dof)
        self.add_connection(connection)

    @modifies_world
    def merge_world_at_pose(self, other: World, pose: cas.TransformationMatrix) -> None:
        """
        Merge another world into the existing one, creates a 6DoF connection between the root of this world and the root
        of the other world.
        :param other: The world to be added.
        :param pose: world_root_T_other_root, the pose of the other world's root with respect to the current world's root
        """
        root_connection = Connection6DoF(parent=self.root, child=other.root, _world=self)
        root_connection.origin = pose
        self.merge_world(other, root_connection)

    def __str__(self):
        return f"{self.__class__.__name__} with {len(self.bodies)} bodies."

    def get_connection(self, parent: Body, child: Body) -> Connection:
        return self.kinematic_structure.get_edge_data(parent.index, child.index)

    def get_body_by_name(self, name: Union[str, PrefixedName]) -> Body:
        """
        Retrieves a body from the list of bodies based on its name.
        If the input is of type `PrefixedName`, it checks whether the prefix is specified and looks for an
        exact match. Otherwise, it matches based on the name's string representation.
        If more than one body with the same name is found, an assertion error is raised.
        If no matching body is found, a `ValueError` is raised.

        :param name: The name of the body to search for. Can be a string or a `PrefixedName` object.
        :return: The `Body` object that matches the given name.
        :raises ValueError: If multiple or no bodies with the specified name are found.
        """
        if isinstance(name, PrefixedName):
            if name.prefix is not None:
                matches = [body for body in self.bodies if body.name == name]
            else:
                matches = [body for body in self.bodies if body.name.name == name.name]
        else:
            matches = [body for body in self.bodies if body.name.name == name]
        if len(matches) > 1:
            raise ValueError(f'Multiple bodies with name {name} found')
        if matches:
            return matches[0]
        raise KeyError(f'Body with name {name} not found')

    def get_degree_of_freedom_by_name(self, name: Union[str, PrefixedName]) -> DegreeOfFreedom:
        """
        Retrieves a DegreeOfFreedom from the list of DegreeOfFreedom based on its name.
        If the input is of type `PrefixedName`, it checks whether the prefix is specified and looks for an
        exact match. Otherwise, it matches based on the name's string representation.
        If more than one body with the same name is found, an assertion error is raised.
        If no matching body is found, a `ValueError` is raised.

        :param name: The name of the DegreeOfFreedom to search for. Can be a string or a `PrefixedName` object.
        :return: The `DegreeOfFreedom` object that matches the given name.
        :raises ValueError: If multiple or no DegreeOfFreedom with the specified name are found.
        """
        if isinstance(name, PrefixedName):
            if name.prefix is not None:
                matches = [dof for dof in self.degrees_of_freedom if dof.name == name]
            else:
                matches = [dof for dof in self.degrees_of_freedom if dof.name.name == name.name]
        else:
            matches = [dof for dof in self.degrees_of_freedom if dof.name.name == name]
        if len(matches) > 1:
            raise ValueError(f'Multiple DegreeOfFreedom with name {name} found')
        if matches:
            return matches[0]
        raise KeyError(f'DegreeOfFreedom with name {name} not found')

    def get_connection_by_name(self, name: Union[str, PrefixedName]) -> Connection:
        """
        Retrieve a connection by its name.
        This method accepts either a string or a `PrefixedName` instance.
        It searches through the list of connections and returns the one
        that matches the given name. If the `PrefixedName` contains a prefix,
        the method ensures the name, including the prefix, matches an existing
        connection. Otherwise, it only considers the unprefixed name. If more than
        one connection matches the specified name, or if no connection is found,
        an exception is raised.

        :param name: The name of the connection to retrieve. Can be a string or
            a `PrefixedName` instance. If a prefix is included in `PrefixedName`,
            it will be used for matching.
        :return: The connection that matches the specified name.
        :raises ValueError: If multiple connections with the given name are found
            or if no connection with the given name exists.
        """
        if isinstance(name, PrefixedName):
            if name.prefix is not None:
                matches = [conn for conn in self.connections if conn.name == name]
            else:
                matches = [conn for conn in self.connections if conn.name.name == name.name]
        else:
            matches = [conn for conn in self.connections if conn.name.name == name]
        if len(matches) > 1:
            raise ValueError(f'Multiple connections with name {name} found')
        if matches:
            return matches[0]
        raise KeyError(f'Connection with name {name} not found')

    @lru_cache(maxsize=None)
    def compute_child_bodies(self, body: Body) -> List[Body]:
        """
        Computes the child bodies of a given body in the world.
        :param body: The body for which to compute child bodies.
        :return: A list of child bodies.
        """
        return list(self.kinematic_structure.successors(body.index))

    def compute_child_bodies_recursive(self, body: Body) -> List[Body]:
        """
        Computes all child bodies of a given body in the world recursively.
        :param body: The body for which to compute child bodies.
        :return: A list of all child bodies.
        """
        children = self.compute_child_bodies(body)
        for child in children:
            children.extend(self.compute_child_bodies_recursive(child))
        return children

    @lru_cache(maxsize=None)
    def compute_parent_body(self, body: Body) -> Body:
        """
        Computes the parent body of a given body in the world.
        :param body: The body for which to compute the parent body.
        :return: The parent body of the given body.
        """
        return next(iter(self.kinematic_structure.predecessors(body.index)))

    @lru_cache(maxsize=None)
    def compute_parent_connection(self, body: Body) -> Connection:
        """
        Computes the parent connection of a given body in the world.
        :param body: The body for which to compute the parent connection.
        :return: The parent connection of the given body.
        """
        return self.kinematic_structure.get_edge_data(self.compute_parent_body(body).index, body.index)

    @lru_cache(maxsize=None)
    def compute_chain_of_bodies(self, root: Body, tip: Body) -> List[Body]:
        if root == tip:
            return [root]
        shortest_paths = rx.all_shortest_paths(self.kinematic_structure, root.index, tip.index, as_undirected=False)

        if len(shortest_paths) == 0:
            raise rx.NoPathFound(f'No path found from {root} to {tip}')

        return [self.kinematic_structure[index] for index in shortest_paths[0]]

    @lru_cache(maxsize=None)
    def compute_chain_of_connections(self, root: Body, tip: Body) -> List[Connection]:
        body_chain = self.compute_chain_of_bodies(root, tip)
        return [self.get_connection(body_chain[i], body_chain[i + 1]) for i in range(len(body_chain) - 1)]

    @lru_cache(maxsize=None)
    def compute_split_chain_of_bodies(self, root: Body, tip: Body) -> Tuple[List[Body], List[Body], List[Body]]:
        """
        Computes the chain between root and tip. Can handle chains that start and end anywhere in the tree.
        :param root: The root body to start the chain from
        :param tip: The tip body to end the chain at
        :return: tuple containing
                    1. chain from root to the common ancestor (excluding common ancestor)
                    2. list containing just the common ancestor
                    3. chain from common ancestor to tip (excluding common ancestor)
        """
        if root == tip:
            return [], [root], []
        root_chain = self.compute_chain_of_bodies(self.root, root)
        tip_chain = self.compute_chain_of_bodies(self.root, tip)
        i = 0
        for i in range(min(len(root_chain), len(tip_chain))):
            if root_chain[i] != tip_chain[i]:
                break
        else:
            i += 1
        common_ancestor = tip_chain[i - 1]
        root_chain = self.compute_chain_of_bodies(common_ancestor, root)
        root_chain = root_chain[1:]
        root_chain = root_chain[::-1]
        tip_chain = self.compute_chain_of_bodies(common_ancestor, tip)
        tip_chain = tip_chain[1:]
        return root_chain, [common_ancestor], tip_chain

    @lru_cache(maxsize=None)
    def compute_split_chain_of_connections(self, root: Body, tip: Body) -> Tuple[List[Connection], List[Connection]]:
        """
        Computes split chains of connections between 'root' and 'tip' bodies. Returns tuple of two Connection lists:
        (root->common ancestor, tip->common ancestor). Returns empty lists if root==tip.

        :param root: The starting `Body` object for the chain of connections.
        :param tip: The ending `Body` object for the chain of connections.
        :return: A tuple of two lists: the first list contains `Connection` objects from the `root` to
            the common ancestor, and the second list contains `Connection` objects from the `tip` to the
            common ancestor.
        """
        if root == tip:
            return [], []
        root_chain, common_ancestor, tip_chain = self.compute_split_chain_of_bodies(root, tip)
        root_chain.append(common_ancestor[0])
        tip_chain.insert(0, common_ancestor[0])
        root_connections = []
        for i in range(len(root_chain) - 1):
            root_connections.append(self.get_connection(root_chain[i + 1], root_chain[i]))
        tip_connections = []
        for i in range(len(tip_chain) - 1):
            tip_connections.append(self.get_connection(tip_chain[i], tip_chain[i + 1]))
        return root_connections, tip_connections

    @property
    def layers(self) -> List[List[Body]]:
        return rx.layers(self.kinematic_structure, [self.root.index], index_output=False)

    def bfs_layout(self, scale: float = 1., align: PlotAlignment = PlotAlignment.VERTICAL) -> Dict[int, np.array]:
        """
        Generate a bfs layout for this circuit.

        :return: A dict mapping the node indices to 2d coordinates.
        """
        layers = self.layers

        pos = None
        nodes = []
        width = len(layers)
        for i, layer in enumerate(layers):
            height = len(layer)
            xs = np.repeat(i, height)
            ys = np.arange(0, height, dtype=float)
            offset = ((width - 1) / 2, (height - 1) / 2)
            layer_pos = np.column_stack([xs, ys]) - offset
            if pos is None:
                pos = layer_pos
            else:
                pos = np.concatenate([pos, layer_pos])
            nodes.extend(layer)

        # Find max length over all dimensions
        pos -= pos.mean(axis=0)
        lim = np.abs(pos).max()  # max coordinate for all axes
        # rescale to (-scale, scale) in all directions, preserves aspect
        if lim > 0:
            pos *= scale / lim

        if align == PlotAlignment.HORIZONTAL:
            pos = pos[:, ::-1]  # swap x and y coords

        pos = dict(zip([node.index for node in nodes], pos))
        return pos

    def plot_kinematic_structure(self, scale: float = 1., align: PlotAlignment = PlotAlignment.VERTICAL) -> None:
        """
        Plots the kinematic structure of the world.
        The plot shows bodies as nodes and connections as edges in a directed graph.
        """
        # Create a new figure
        plt.figure(figsize=(12, 8))

        pos = self.bfs_layout(scale=scale, align=align)

        rustworkx.visualization.mpl_draw(self.kinematic_structure, pos=pos, labels=lambda body: str(body.name),
                                         with_labels=True,
                                         edge_labels=lambda edge: edge.__class__.__name__)

        plt.title("World Kinematic Structure")
        plt.axis('off')  # Hide axes
        plt.show()

    def _travel_branch(self, body: Body, visitor: rustworkx.visit.DFSVisitor) -> None:
        """
        Apply a DFS Visitor to a subtree of the kinematic structure.

        :param body: Starting point of the search
        :param visitor: This visitor to apply.
        """
        rx.dfs_search(self.kinematic_structure, [body.index], visitor)

    def compile_forward_kinematics_expressions(self) -> None:
        """
        Traverse the kinematic structure and compile forward kinematics expressions for fast evaluation.
        """
        new_fks = ForwardKinematicsVisitor(self)
        self._travel_branch(self.root, new_fks)
        new_fks.compile_forward_kinematics()
        self._fk_computer = new_fks

    def _recompute_forward_kinematics(self) -> None:
        self._fk_computer.recompute()

    @copy_lru_cache()
    def compose_forward_kinematics_expression(self, root: Body, tip: Body) -> cas.TransformationMatrix:
        """
        :param root: The root body in the kinematic chain.
            It determines the starting point of the forward kinematics calculation.
        :param tip: The tip body in the kinematic chain.
            It determines the endpoint of the forward kinematics calculation.
        :return: An expression representing the computed forward kinematics of the tip body relative to the root body.
        """

        fk = cas.TransformationMatrix()
        root_chain, tip_chain = self.compute_split_chain_of_connections(root, tip)
        connection: Connection
        for connection in root_chain:
            tip_T_root = connection.origin_expression.inverse()
            fk = fk.dot(tip_T_root)
        for connection in tip_chain:
            fk = fk.dot(connection.origin_expression)
        fk.reference_frame = root
        fk.child_frame = tip
        return fk

    def compute_forward_kinematics(self, root: Body, tip: Body) -> cas.TransformationMatrix:
        """
        Compute the forward kinematics from the root body to the tip body.

        Calculate the transformation matrix representing the pose of the
        tip body relative to the root body.

        :param root: Root body for which the kinematics are computed.
        :param tip: Tip body to which the kinematics are computed.
        :return: Transformation matrix representing the relative pose of the tip body with respect to the root body.
        """
        return cas.TransformationMatrix(self.compute_forward_kinematics_np(root, tip))

    def compute_forward_kinematics_np(self, root: Body, tip: Body) -> NpMatrix4x4:
        """
        Compute the forward kinematics from the root body to the tip body, root_T_tip and return it as a 4x4 numpy ndarray.

        Calculate the transformation matrix representing the pose of the
        tip body relative to the root body, expressed as a numpy ndarray.

        :param root: Root body for which the kinematics are computed.
        :param tip: Tip body to which the kinematics are computed.
        :return: Transformation matrix representing the relative pose of the tip body with respect to the root body.
        """
        return self._fk_computer.compute_forward_kinematics_np(root, tip).copy()

    def transform(self, spatial_object: cas.SpatialType, target_frame: Body) -> cas.SpatialType:
        """
        Transform a given spatial object from its reference frame to a target frame.

        Calculate the transformation from the reference frame of the provided
        spatial object to the specified target frame. Apply the transformation
        differently depending on the type of the spatial object:

        - If the object is a Quaternion, compute its rotation matrix, transform it, and
          convert back to a Quaternion.
        - For other types, apply the transformation matrix directly.

        :param spatial_object: The spatial object to be transformed.
        :param target_frame: The target body frame to which the spatial object should
            be transformed.
        :return: The spatial object transformed to the target frame. If the input object
            is a Quaternion, the returned object is a Quaternion. Otherwise, it is the
            transformed spatial object.
        """
        target_frame_T_reference_frame = self.compute_forward_kinematics(root=target_frame,
                                                                         tip=spatial_object.reference_frame)
        if isinstance(spatial_object, cas.Quaternion):
            reference_frame_R = spatial_object.to_rotation_matrix()
            target_frame_R = target_frame_T_reference_frame @ reference_frame_R
            return target_frame_R.to_quaternion()
        else:
            return target_frame_T_reference_frame @ spatial_object

    def find_dofs_for_position_symbols(self, symbols: List[cas.Symbol]) -> List[DegreeOfFreedom]:
        result = []
        for s in symbols:
            for dof in self.degrees_of_freedom:
                if s == dof.symbols.position:
                    result.append(dof)
        return result

    def compute_inverse_kinematics(self, root: Body, tip: Body, target: cas.TransformationMatrix,
                                   dt: float = 0.05, max_iterations: int = 200,
                                   translation_velocity: float = 0.2, rotation_velocity: float = 0.2) \
            -> Dict[DegreeOfFreedom, float]:
        """
        Compute inverse kinematics using quadratic programming.

        :param root: Root body of the kinematic chain
        :param tip: Tip body of the kinematic chain
        :param target: Desired tip pose relative to the root body
        :param dt: Time step for integration
        :param max_iterations: Maximum number of iterations
        :param translation_velocity: Maximum translation velocity
        :param rotation_velocity: Maximum rotation velocity
        :return: Dictionary mapping DOF names to their computed positions
        """
        ik_solver = InverseKinematicsSolver(self)
        return ik_solver.solve(root, tip, target, dt, max_iterations, translation_velocity, rotation_velocity)

    def apply_control_commands(self, commands: np.ndarray, dt: float, derivative: Derivatives) -> None:
        """
        Updates the state of a system by applying control commands at a specified derivative level,
        followed by backward integration to update lower derivatives.

        :param commands: Control commands to be applied at the specified derivative
            level. The array length must match the number of free variables
            in the system.
        :param dt: Time step used for the integration of lower derivatives.
        :param derivative: The derivative level to which the control commands are
            applied.
        :return: None
        """
        if len(commands) != len(self.degrees_of_freedom):
            raise ValueError(
                f"Commands length {len(commands)} does not match number of free variables {len(self.degrees_of_freedom)}")

        self.state.set_derivative(derivative, commands)

        for i in range(derivative - 1, -1, -1):
            self.state.set_derivative(i, self.state.get_derivative(i) + self.state.get_derivative(i + 1) * dt)
        for connection in self.connections:
            if isinstance(connection, HasUpdateState):
                connection.update_state(dt)
        self.notify_state_change()

    def set_positions_1DOF_connection(self, new_state: Dict[Has1DOFState, float]) -> None:
        for connection, value in new_state.items():
            connection.position = value
        self.notify_state_change()
