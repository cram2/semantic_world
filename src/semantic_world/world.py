from __future__ import annotations

from dataclasses import dataclass, field
from functools import wraps
from typing import Dict, Tuple, OrderedDict, Union

import networkx as nx
import numpy as np
from typing_extensions import List

import semantic_world.spatial_types.spatial_types as cas
from .connections import FixedConnection
from .free_variable import FreeVariable
from .prefixed_name import PrefixedName
from .spatial_types.derivatives import Derivatives
from .spatial_types.math import inverse_frame
from .spatial_types.symbol_manager import symbol_manager
from .utils import IDGenerator
from .world_entity import Body, Connection

id_generator = IDGenerator()


class WorldVisitor:
    def link_call(self, body: Body) -> bool:
        """
        :return: return True to stop climbing up the branch
        """
        return False

    def connection_call(self, connection: Connection) -> bool:
        """
        :return: return True to stop climbing up the branch
        """
        return False


class ResetJointStateContextManager:
    def __init__(self, world: World):
        self.world = world

    def __enter__(self):
        self.position_state = self.world.position_state.copy()
        self.velocity_state = self.world.velocity_state.copy()
        self.acceleration_state = self.world.acceleration_state.copy()
        self.jerk_state = self.world.jerk_state.copy()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is None:
            self.world.position_state = self.position_state
            self.world.velocity_state = self.velocity_state
            self.world.acceleration_state = self.acceleration_state
            self.world.jerk_state = self.jerk_state
            self.world.notify_state_change()


class WorldModelUpdateContextManager:
    first: bool = True

    def __init__(self, world: World):
        self.world = world

    def __enter__(self):
        if self.world.context_manager_active:
            self.first = False
        self.world.context_manager_active = True
        return self.world

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.first:
            self.world.context_manager_active = False
            if exc_type is None:
                self.world._notify_model_change()


def modifies_world(func):
    @wraps(func)
    def wrapper(self: World, *args, **kwargs):
        with self.modify_world():
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

    root: Body = field(default=Body(name=PrefixedName(prefix="world", name="root")), kw_only=True)
    """
    The root link of the world.
    """

    kinematic_structure: nx.DiGraph = field(default_factory=nx.DiGraph, kw_only=True, repr=False)
    """
    The kinematic structure of the world.
    The kinematic structure is a tree-like directed graph where the nodes represent bodies in the world,
    and the edges represent connections between them.
    """

    free_variables: Dict[PrefixedName, FreeVariable] = field(default_factory=dict)

    position_state: list = field(default_factory=list)
    velocity_state: list = field(default_factory=list)
    acceleration_state: list = field(default_factory=list)
    jerk_state: list = field(default_factory=list)

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

    context_manager_active: bool = False

    def __post_init__(self):
        self.add_body(self.root)

    @modifies_world
    def create_free_variable(self,
                             name: PrefixedName,
                             lower_limits: Dict[Derivatives, float],
                             upper_limits: Dict[Derivatives, float]) -> FreeVariable:
        free_variable = FreeVariable(name=name,
                                     lower_limits=lower_limits,
                                     upper_limits=upper_limits,
                                     world=self)
        initial_value = 0
        if free_variable.has_position_limits():
            lower_limit = free_variable.get_lower_limit(derivative=Derivatives.position,
                                                        evaluated=True)
            upper_limit = free_variable.get_upper_limit(derivative=Derivatives.position,
                                                        evaluated=True)
            initial_value = min(max(0, lower_limit), upper_limit)
        self.position_state.append(initial_value)
        self.free_variables[name] = free_variable
        return free_variable

    def validate(self):
        """
        Validate the world.

        The world must be a tree.
        """
        if not nx.is_tree(self.kinematic_structure):
            raise ValueError("The world is not a tree.")

    def modify_world(self):
        return WorldModelUpdateContextManager(self)

    def reset_joint_state_context(self):
        return ResetJointStateContextManager(self)

    def notify_state_change(self):
        """
        If you have changed the state of the world, call this function to trigger necessary events and increase
        the state version.
        """
        # clear_memo(self.compute_fk)
        # clear_memo(self.compute_fk_with_collision_offset_np)
        self._recompute_fks()
        self._state_version += 1

    def _notify_model_change(self):
        """
        Call this function if you have changed the model of the world to trigger necessary events and increase
        the model version number.
        """
        if not self.context_manager_active:
            # self._fix_tree_structure()
            # self.reset_cache()
            self.init_all_fks()
            # self._cleanup_unused_free_variable()
            self.notify_state_change()
            self._model_version += 1

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
        return [self.kinematic_structure.get_edge_data(*edge)[Connection.__name__]
                for edge in self.kinematic_structure.edges()]

    @modifies_world
    def add_body(self, body: Body):
        """
        Add a body to the world.

        :param body: The body to add.
        """
        self.kinematic_structure.add_node(body)
        body._world = self
        self._model_version += 1

    @modifies_world
    def add_connection(self, connection: Connection):
        """
        Add a connection to the world.

        :param connection: The connection to add.
        """
        self.add_body(connection.parent)
        self.add_body(connection.child)
        kwargs = {Connection.__name__: connection}
        self.kinematic_structure.add_edge(connection.parent, connection.child, **kwargs)
        self._model_version += 1

    def get_connection(self, parent: Body, child: Body) -> Connection:
        return self.kinematic_structure.get_edge_data(parent, child)[Connection.__name__]

    def get_body_by_name(self, name: Union[str, PrefixedName]) -> Body:
        if isinstance(name, PrefixedName):
            if name.prefix is not None:
                matches = [body for body in self.bodies if body.name == name]
            else:
                matches = [body for body in self.bodies if body.name.name == name.name]
        else:
            matches = [body for body in self.bodies if body.name.name == name]
        assert len(matches) <= 1, f'Multiple bodies with name {name} found'
        if matches:
            return matches[0]
        raise ValueError(f'Body with name {name} not found')

    def get_connection_by_name(self, name: str) -> Connection:
        return [c for c in self.connections if c.origin.reference_frame.name == name][0]

    def get_connection_by_prefix_name(self, name: PrefixedName) -> Connection:
        return [c for c in self.connections if c.origin.reference_frame == name][0]

    # @memoize
    def compute_chain(self,
                      root_link_name: PrefixedName,
                      tip_link_name: PrefixedName,
                      add_joints: bool,
                      add_links: bool,
                      add_fixed_joints: bool,
                      add_non_controlled_joints: bool) -> List[Union[Body, Connection]]:
        """
        Computes a chain between root_link_name and tip_link_name. Only works if root_link_name is above tip_link_name
        in the world.
        """
        if root_link_name == tip_link_name:
            return []

        root_body = self.get_body_by_name(root_link_name)
        tip_body = self.get_body_by_name(tip_link_name)

        if not (root_body in self.bodies and tip_body in self.bodies):
            raise ValueError("Root or tip link not found in world")

        try:
            path = nx.shortest_path(self.kinematic_structure, root_body, tip_body)
        except nx.NetworkXNoPath:
            raise ValueError(f"No path found between {root_link_name} and {tip_link_name}")

        result = []
        for i in range(len(path) - 1):
            if add_links:
                result.append(path[i])
            if add_joints:
                connection = self.get_connection(path[i], path[i + 1])
                if isinstance(connection, FixedConnection):
                    if add_fixed_joints:
                        result.append(connection)
                else:
                    if add_non_controlled_joints:
                        result.append(connection)

        if add_links:
            result.append(path[-1])

        return result

    # @memoize
    def compute_split_chain(self,
                            root_link_name: PrefixedName,
                            tip_link_name: PrefixedName,
                            add_joints: bool,
                            add_links: bool,
                            add_fixed_joints: bool,
                            add_non_controlled_joints: bool) \
            -> Tuple[List[Union[Body, Connection]], List[Union[Body, Connection]], List[Union[Body, Connection]]]:
        """
        Computes the chain between root_link_name and tip_link_name. Can handle chains that start and end anywhere
        in the tree.
        :param root_link_name:
        :param tip_link_name:
        :param add_joints:
        :param add_links:
        :param add_fixed_joints: only used if add_joints == True
        :param add_non_controlled_joints: only used if add_joints == True
        :return: tuple containing
                    1. chain from root_link_name to the connecting link
                    2. the connecting link, if add_lins is True
                    3. chain from connecting link to tip_link_name
        """
        if root_link_name == tip_link_name:
            return [], [], []
        root_chain = self.compute_chain(self.root.name, root_link_name, False, True, True, True)
        tip_chain = self.compute_chain(self.root.name, tip_link_name, False, True, True, True)
        for i in range(min(len(root_chain), len(tip_chain))):
            if root_chain[i] != tip_chain[i]:
                break
        else:
            i += 1
        connection = tip_chain[i - 1]
        root_chain = self.compute_chain(connection.name, root_link_name, add_joints, add_links, add_fixed_joints,
                                        add_non_controlled_joints)
        if add_links:
            root_chain = root_chain[1:]
        root_chain = root_chain[::-1]
        tip_chain = self.compute_chain(connection.name, tip_link_name, add_joints, add_links, add_fixed_joints,
                                       add_non_controlled_joints)
        if add_links:
            tip_chain = tip_chain[1:]
        return root_chain, [connection] if add_links else [], tip_chain

    def plot_kinematic_structure(self):
        """
        Plots the kinematic structure of the world.
        The plot shows bodies as nodes and connections as edges in a directed graph.
        """
        import matplotlib.pyplot as plt

        # Create a new figure
        plt.figure(figsize=(12, 8))

        # Use spring layout for node positioning
        pos = nx.drawing.bfs_layout(self.kinematic_structure, start=self.root)

        # Draw nodes (bodies)
        nx.draw_networkx_nodes(self.kinematic_structure, pos,
                               node_color='lightblue',
                               node_size=2000)

        # Draw edges (connections)
        edges = self.kinematic_structure.edges(data=True)
        nx.draw_networkx_edges(self.kinematic_structure, pos,
                               edge_color='gray',
                               arrows=True,
                               arrowsize=50)

        # Add link names as labels
        labels = {node: node.name.name for node in self.kinematic_structure.nodes()}
        nx.draw_networkx_labels(self.kinematic_structure, pos, labels)

        # Add joint types as edge labels
        edge_labels = {(edge[0], edge[1]): edge[2][Connection.__name__].__class__.__name__
                       for edge in self.kinematic_structure.edges(data=True)}
        nx.draw_networkx_edge_labels(self.kinematic_structure, pos, edge_labels)

        plt.title("World Kinematic Structure")
        plt.axis('off')  # Hide axes
        plt.show()

    def _travel_branch(self, body: Body, companion: WorldVisitor):
        """
        Do a depth first search on a branch starting at link_name.
        Use companion to do whatever you want. It link_call and joint_call are called on every link/joint it sees.
        The traversion is stopped once they return False.
        :param body: starting point of the search
        :param companion: payload. Implement your own WorldVisitor for your purpose.
        """
        if companion.link_call(body):
            return

        for _, child_body, edge_data in self.kinematic_structure.edges(body, data=True):
            connection = edge_data[Connection.__name__]
            if companion.connection_call(connection):
                continue
            self._travel_branch(child_body, companion)

    def init_all_fks(self):
        class FKVisitor(WorldVisitor):
            idx_start: Dict[PrefixedName, int]
            compiled_collision_fks: cas.CompiledFunction
            compiled_all_fks: cas.CompiledFunction
            str_params: List[str]
            fks: np.ndarray
            fks_exprs: Dict[PrefixedName, cas.TransformationMatrix]

            def __init__(self, world: World):
                self.world = world
                self.fks_exprs = {self.world.root.name: cas.TransformationMatrix()}
                self.tf = OrderedDict()

            def connection_call(self, connection: Connection) -> bool:
                map_T_parent = self.fks_exprs[connection.parent.name]
                self.fks_exprs[connection.child.name] = map_T_parent.dot(connection.origin)
                self.tf[(connection.parent.name, connection.child.name)] = connection.parent_T_child_as_pos_quaternion()
                return False

            def compile_fks(self):
                all_fks = cas.vstack([self.fks_exprs[body.name] for body in self.world.bodies])
                tf = cas.vstack([pose for pose in self.tf.values()])
                collision_fks = []
                for body in sorted(self.world.bodies_with_collisions, key=lambda body: body.name):
                    if body == self.world.root:
                        continue
                    collision_fks.append(self.fks_exprs[body.name])
                collision_fks = cas.vstack(collision_fks)
                params = set()
                params.update(all_fks.free_symbols())
                params.update(collision_fks.free_symbols())
                params = list(params)
                self.str_params = [str(v) for v in params]
                self.compiled_all_fks = all_fks.compile(parameters=params)
                self.compiled_collision_fks = collision_fks.compile(parameters=params)
                self.compiled_tf = tf.compile(parameters=params)
                self.idx_start = {body.name: i * 4 for i, body in enumerate(self.world.bodies)}

            def recompute(self):
                # self.compute_fk_np.memo.clear()
                self.subs = symbol_manager.resolve_symbols(self.compiled_all_fks.params)
                self.fks = self.compiled_all_fks.fast_call(self.subs)

            def compute_tf(self):
                return self.compiled_tf.fast_call(self.subs)

            # @memoize
            def compute_fk_np(self, root: PrefixedName, tip: PrefixedName) -> np.ndarray:
                root_is_world = root == self.world.root.name
                tip_is_world = tip == self.world.root.name

                if not tip_is_world:
                    i = self.idx_start[tip]
                    map_T_tip = self.fks[i:i + 4]
                    if root_is_world:
                        return map_T_tip

                if not root_is_world:
                    i = self.idx_start[root]
                    map_T_root = self.fks[i:i + 4]
                    root_T_map = inverse_frame(map_T_root)
                    if tip_is_world:
                        return root_T_map

                if tip_is_world and root_is_world:
                    return np.eye(4)

                return root_T_map @ map_T_tip

        new_fks = FKVisitor(self)
        self._travel_branch(self.root, new_fks)
        new_fks.compile_fks()
        self._fk_computer = new_fks

    def _recompute_fks(self):
        self._fk_computer.recompute()

    def compute_fk_np(self, root: PrefixedName, tip: PrefixedName) -> np.ndarray:
        return self._fk_computer.compute_fk_np(root, tip)
