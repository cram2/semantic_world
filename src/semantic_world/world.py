from __future__ import annotations

from dataclasses import dataclass, field
from functools import wraps
from typing import Dict, Tuple, OrderedDict, Union, Optional

import networkx as nx
import numpy as np
from typing_extensions import List

from .spatial_types import spatial_types as cas
from .connections import HasUpdateState
from .degree_of_freedom import DegreeOfFreedom
from .prefixed_name import PrefixedName
from .spatial_types.derivatives import Derivatives
from .spatial_types.math import inverse_frame
from .utils import IDGenerator, memoize, clear_memo, copy_memoize
from .world_entity import Body, Connection

id_generator = IDGenerator()


class WorldVisitor:
    def body_call(self, body: Body) -> bool:
        """
        :return: return True to stop climbing up the branch
        """
        return False

    def connection_call(self, connection: Connection) -> bool:
        """
        :return: return True to stop climbing up the branch
        """
        return False


class ForwardKinematicsVisitor(WorldVisitor):
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

    def connection_call(self, connection: Connection) -> bool:
        """
        Gathers forward kinematics expressions for a connection.
        """
        map_T_parent = self.child_body_to_fk_expr[connection.parent.name]
        self.child_body_to_fk_expr[connection.child.name] = map_T_parent.dot(connection.origin)
        self.tf[(connection.parent.name, connection.child.name)] = connection.origin_as_position_quaternion()
        return False

    def compile_forward_kinematics(self) -> None:
        """
        Compiles forward kinematics expressions for fast evaluation.
        """
        all_fks = cas.vstack([self.child_body_to_fk_expr[body.name] for body in self.world.bodies])
        tf = cas.vstack([pose for pose in self.tf.values()])
        collision_fks = []
        for body in sorted(self.world.bodies_with_collisions, key=lambda body: body.name):
            if body == self.world.root:
                continue
            collision_fks.append(self.child_body_to_fk_expr[body.name])
        collision_fks = cas.vstack(collision_fks)
        params = [v.get_symbol(Derivatives.position) for v in self.world.degrees_of_freedom.values()]
        self.compiled_all_fks = all_fks.compile(parameters=params)
        self.compiled_collision_fks = collision_fks.compile(parameters=params)
        self.compiled_tf = tf.compile(parameters=params)
        self.idx_start = {body.name: i * 4 for i, body in enumerate(self.world.bodies)}

    def recompute(self) -> None:
        """
        Clears cache and recomputes all forward kinematics. Should be called after a state update.
        """
        self.compute_forward_kinematics_np.memo.clear()
        self.subs = self.world.state[Derivatives.position]
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

    @memoize
    def compute_forward_kinematics_np(self, root: Body, tip: Body) -> np.ndarray:
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
        self.state = self.world.state.copy()

    def __exit__(self, exc_type: Optional[type], exc_val: Optional[Exception], exc_tb: Optional[type]) -> None:
        if exc_type is None:
            self.world.state = self.state
            self.world.notify_state_change()


class WorldModelUpdateContextManager:
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
    The root body of the world.
    """

    kinematic_structure: nx.DiGraph = field(default_factory=nx.DiGraph, kw_only=True, repr=False)
    """
    The kinematic structure of the world.
    The kinematic structure is a tree-like directed graph where the nodes represent bodies in the world,
    and the edges represent connections between them.
    """

    degrees_of_freedom: Dict[PrefixedName, DegreeOfFreedom] = field(default_factory=dict)

    state: np.ndarray = field(default_factory=lambda: np.empty((4, 0), dtype=float))
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

    def __post_init__(self):
        self.add_body(self.root)

    def __hash__(self):
        return hash(id(self))

    @modifies_world
    def create_degree_of_freedom(self,
                                 name: PrefixedName,
                                 lower_limits: Optional[Dict[Derivatives, float]] = None,
                                 upper_limits: Optional[Dict[Derivatives, float]] = None) -> DegreeOfFreedom:
        """
        Create a degree of freedom in the world and return it.
        For dependent kinematics, DoFs must be created with this method and passed to the connection's conctructor.
        :param name: Name of the DoF.
        :param lower_limits: If the DoF is actively controlled, it must have at least velocity limits.
        :param upper_limits: If the DoF is actively controlled, it must have at least velocity limits.
        :return: The already registered DoF.
        """
        dof = DegreeOfFreedom(name=name,
                              _lower_limits=lower_limits,
                              _upper_limits=upper_limits,
                              _world=self)
        initial_position = 0
        lower_limit = dof.get_lower_limit(derivative=Derivatives.position)
        if lower_limit is not None:
            initial_position = max(lower_limit, initial_position)
        upper_limit = dof.get_upper_limit(derivative=Derivatives.position)
        if upper_limit is not None:
            initial_position = min(upper_limit, initial_position)
        full_initial_state = np.array([initial_position, 0, 0, 0], dtype=float).reshape((4, 1))
        self.state = np.hstack((self.state, full_initial_state))
        self.degrees_of_freedom[name] = dof
        return dof

    def validate(self) -> None:
        """
        Validate the world.

        The world must be a tree.
        """
        if not nx.is_tree(self.kinematic_structure):
            raise ValueError("The world is not a tree.")

    def modify_world(self) -> WorldModelUpdateContextManager:
        return WorldModelUpdateContextManager(self)

    def reset_state_context(self) -> ResetStateContextManager:
        return ResetStateContextManager(self)

    def reset_cache(self) -> None:
        # super().reset_cache()
        # clear_memo(self.get_directly_controlled_child_links_with_collisions)
        # clear_memo(self.get_directly_controlled_child_links_with_collisions)
        # clear_memo(self.compute_chain_reduced_to_controlled_joints)
        # clear_memo(self.get_movable_parent_joint)
        # clear_memo(self.get_controlled_parent_joint_of_link)
        # clear_memo(self.get_controlled_parent_joint_of_joint)
        clear_memo(self.compute_split_chain_of_bodies)
        clear_memo(self.compute_split_chain_of_connections)
        # clear_memo(self.are_linked)
        # clear_memo(self.compose_fk_expression)
        clear_memo(self.compute_chain_of_bodies)
        clear_memo(self.compute_chain_of_connections)
        # clear_memo(self.is_link_controlled)
        for dof in self.degrees_of_freedom.values():
            dof.reset_cache()

    def notify_state_change(self) -> None:
        """
        If you have changed the state of the world, call this function to trigger necessary events and increase
        the state version.
        """
        # clear_memo(self.compute_fk)
        # clear_memo(self.compute_fk_with_collision_offset_np)
        self._recompute_forward_kinematics()
        self._state_version += 1

    def _notify_model_change(self) -> None:
        """
        Call this function if you have changed the model of the world to trigger necessary events and increase
        the model version number.
        """
        if not self.world_is_being_modified:
            # self._fix_tree_structure()
            self.reset_cache()
            self.compile_forward_kinematics_expressions()
            # self._cleanup_unused_dofs()
            self.notify_state_change()
            self._model_version += 1
            self.validate()

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
    def add_body(self, body: Body) -> None:
        """
        Add a body to the world.

        :param body: The body to add.
        """
        self.kinematic_structure.add_node(body)
        body._world = self

    @modifies_world
    def add_connection(self, connection: Connection) -> None:
        """
        Add a connection AND the bodies it connects to the world.

        :param connection: The connection to add.
        """
        self.add_body(connection.parent)
        self.add_body(connection.child)
        kwargs = {Connection.__name__: connection}
        connection._world = self
        self.kinematic_structure.add_edge(connection.parent, connection.child, **kwargs)

    @modifies_world
    def merge_world(self, world: World) -> None:
        """
        Merge a world into the existing one by merging degrees of freedom, states, connections, and bodies.

        :param world: The world to be added. 
        :return: None
        """
        for dof in world.degrees_of_freedom.values():
            dof.state_idx += len(self.degrees_of_freedom)
        self.degrees_of_freedom.update(world.degrees_of_freedom)
        self.state = np.hstack((self.state, world.state))
        for connection in world.connections:
            self.add_connection(connection)

    def get_connection(self, parent: Body, child: Body) -> Connection:
        return self.kinematic_structure.get_edge_data(parent, child)[Connection.__name__]

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
        raise ValueError(f'Body with name {name} not found')

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
        raise ValueError(f'Connection with name {name} not found')

    def compute_child_bodies(self, body: Body) -> List[Body]:
        """
        Computes the child bodies of a given body in the world.
        :param body: The body for which to compute child bodies.
        :return: A list of child bodies.
        """
        return list(self.kinematic_structure.successors(body))

    def compute_parent_body(self, body: Body) -> Body:
        """
        Computes the parent body of a given body in the world.
        :param body: The body for which to compute the parent body.
        :return: The parent body of the given body.
        """
        return next(self.kinematic_structure.predecessors(body))

    def compute_parent_connection(self, body: Body) -> Connection:
        """
        Computes the parent connection of a given body in the world.
        :param body: The body for which to compute the parent connection.
        :return: The parent connection of the given body.
        """
        return self.kinematic_structure.get_edge_data(self.compute_parent_body(body), body)[Connection.__name__]

    @memoize
    def compute_chain_of_bodies(self, root: Body, tip: Body) -> List[Body]:
        return nx.shortest_path(self.kinematic_structure, root, tip)

    @memoize
    def compute_chain_of_connections(self, root: Body, tip: Body) -> List[Connection]:
        body_chain = self.compute_chain_of_bodies(root, tip)
        return [self.get_connection(body_chain[i], body_chain[i + 1]) for i in range(len(body_chain) - 1)]

    @memoize
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

    @memoize
    def compute_split_chain_of_connections(self, root: Body, tip: Body) \
            -> Tuple[List[Connection], List[Connection]]:
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

    def plot_kinematic_structure(self) -> None:
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

    def _travel_branch(self, body: Body, visitor: WorldVisitor) -> None:
        """
        Do a depth first search on a branch starting at body.
        Use visitor to do whatever you want. It body_call and connection_call are called on every body/connection it sees.
        The traversion is stopped once they return False.
        :param body: starting point of the search
        :param visitor: payload. Implement your own WorldVisitor for your purpose.
        """
        if visitor.body_call(body):
            return

        for _, child_body, edge_data in self.kinematic_structure.edges(body, data=True):
            connection = edge_data[Connection.__name__]
            if visitor.connection_call(connection):
                continue
            self._travel_branch(child_body, visitor)

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

    @copy_memoize
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
            tip_T_root = connection.origin.inverse()
            fk = fk.dot(tip_T_root)
        for connection in tip_chain:
            fk = fk.dot(connection.origin)
        fk.reference_frame = root.name
        fk.child_frame = tip.name
        return fk

    def compute_forward_kinematics_np(self, root: Body, tip: Body) -> np.ndarray:
        """
        Computes the forward kinematics from the root body to the tip body, root_T_tip.

        This method computes the transformation matrix representing the pose of the
        tip body relative to the root body, expressed as a numpy ndarray.

        :param root: Root body for which the kinematics are computed.
        :param tip: Tip body to which the kinematics are computed.
        :return: Transformation matrix representing the relative pose of the tip body with respect to the root body.
        """
        return self._fk_computer.compute_forward_kinematics_np(root, tip)

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

        self.state[derivative] = commands

        for i in range(derivative - 1, -1, -1):
            self.state[i] += self.state[i + 1] * dt
        for connection in self.connections:
            if isinstance(connection, HasUpdateState):
                connection.update_state(dt)
        self.notify_state_change()
