from __future__ import annotations

import logging
import os
from abc import abstractmethod, ABC
from collections import defaultdict
from dataclasses import dataclass, field
from functools import lru_cache, cached_property
from itertools import combinations_with_replacement
from typing import Tuple, Iterable, Set, Dict, Union

from lxml import etree
import rustworkx as rx
from typing_extensions import Optional, List, Self

from .connections import ActiveConnection, FixedConnection, OmniDrive
from .prefixed_name import PrefixedName
from .spatial_types.spatial_types import Vector3
from .world import World
from .world_entity import Body, RootedView, Connection, View


@dataclass
class RobotView(RootedView, ABC):
    """
    Represents a collection of connected robot bodies, starting from a root body, and ending in a unspecified collection
    of tip bodies.
    """
    _robot: AbstractRobot = field(default=None)
    """
    The robot this view belongs to
    """

    def __post_init__(self):
        if self._world is not None:
            self._world.add_view(self)

    @abstractmethod
    def assign_to_robot(self, robot: AbstractRobot):
        """
        This method assigns the robot to the current view, and then iterates through its own fields to call the
        appropriate methods to att them to the robot.

        :param robot: The robot to which this view should be assigned.
        """
        ...


@dataclass
class KinematicChain(RobotView, ABC):
    """
    Abstract base class for kinematic chain in a robot, starting from a root body, and ending in a specific tip body.
    A kinematic chain can contain both a manipulator and sensors at the same time. There are no assumptions about the
    position of the manipulator or sensors in the kinematic chain
    """
    tip: Body = field(default_factory=Body)
    """
    The tip body of the kinematic chain, which is the last body in the chain.
    """

    manipulator: Optional[Manipulator] = None
    """
    The manipulator of the kinematic chain, if it exists. This is usually a gripper or similar device.
    """

    sensors: Set[Sensor] = field(default_factory=set)
    """
    A collection of sensors in the kinematic chain, such as cameras or other sensors.
    """

    @property
    def bodies(self) -> Iterable[Body]:
        """
        Returns itself as a kinematic chain.
        """
        return self._world.compute_chain_of_bodies(self.root, self.tip)

    @property
    def connections(self) -> Iterable[Connection]:
        """
        Returns the connections of the kinematic chain.
        This is a list of connections between the bodies in the kinematic chain
        """
        return self._world.compute_chain_of_connections(self.root, self.tip)

    def assign_to_robot(self, robot: AbstractRobot):
        """
        Assigns the kinematic chain to the given robot. This method ensures that the kinematic chain is only assigned
        to one robot at a time, and raises an error if it is already assigned to another robot.
        """
        if self._robot is not None and self._robot != robot:
            raise ValueError(f"Kinematic chain {self.name} is already part of another robot: {self._robot.name}.")
        if self._robot is not None and self._robot == robot:
            return
        self._robot = robot
        if self.manipulator is not None:
            robot.add_manipulator(self.manipulator)
        for sensor in self.sensors:
            robot.add_sensor(sensor)

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tip))


@dataclass
class Arm(KinematicChain):
    """
    Represents an arm of a robot, which is a kinematic chain with a specific tip body.
    An arm has a manipulators and potentially sensors.
    """

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tip))


@dataclass
class Manipulator(RobotView, ABC):
    """
    Abstract base class of robot manipulators. Always has a tool frame.
    """
    tool_frame: Body = field(default_factory=Body)

    def assign_to_robot(self, robot: AbstractRobot):
        """
        Assigns the manipulator to the given robot. This method ensures that the manipulator is only assigned
        to one robot at a time, and raises an error if it is already assigned to another robot.
        """
        if self._robot is not None and self._robot != robot:
            raise ValueError(f"Manipulator {self.name} is already part of another robot: {self._robot.name}.")
        if self._robot is not None and self._robot == robot:
            return
        self._robot = robot

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tool_frame))


@dataclass
class Finger(KinematicChain):
    """
    A finger is a kinematic chain, since it should have an unambiguous tip body, and may contain sensors.
    """

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tip))


@dataclass
class ParallelGripper(Manipulator):
    """
    Represents a gripper of a robot. Contains a collection of fingers and a thumb. The thumb is a specific finger
    that always needs to touch an object when grasping it, ensuring a stable grasp.
    """
    finger: Finger = field(default_factory=Finger)
    thumb: Finger = field(default_factory=Finger)

    def assign_to_robot(self, robot: AbstractRobot):
        """
        Assigns the parallel gripper to the given robot and calls the appropriate methods for the its finger and thumb.
         This method ensures that the parallel gripper is only assigned to one robot at a time, and raises an error if
         it is already assigned to another
        """
        if self._robot is not None and self._robot != robot:
            raise ValueError(f"ParallelGripper {self.name} is already part of another robot: {self._robot.name}.")
        if self._robot is not None and self._robot == robot:
            return
        self._robot = robot
        robot.add_kinematic_chain(self.finger)
        robot.add_kinematic_chain(self.thumb)

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tool_frame))


@dataclass
class Sensor(RobotView, ABC):
    """
    Abstract base class for any kind of sensor in a robot.
    """

    def assign_to_robot(self, robot: AbstractRobot):
        """
        Assigns the sensor to the given robot. This method ensures that the sensor is only assigned
        to one robot at a time, and raises an error if it is already assigned to another robot.
        """
        if self._robot is not None and self._robot != robot:
            raise ValueError(f"Sensor {self.name} is already part of another robot: {self._robot.name}.")
        if self._robot is not None and self._robot == robot:
            return
        self._robot = robot


@dataclass
class FieldOfView:
    """
    Represents the field of view of a camera sensor, defined by the vertical and horizontal angles of the camera's view.
    """
    vertical_angle: float
    horizontal_angle: float


@dataclass
class Camera(Sensor):
    """
    Represents a camera sensor in a robot.
    """
    forward_facing_axis: Vector3 = field(default_factory=Vector3)
    field_of_view: FieldOfView = field(default_factory=FieldOfView)
    minimal_height: float = 0.0
    maximal_height: float = 1.0

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root))


@dataclass
class Neck(KinematicChain):
    """
    Represents a special kinematic chain that connects the head of a robot with a collection of sensors, such as cameras
    and which does not have a manipulator.
    """

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tip))


@dataclass
class Torso(KinematicChain):
    """
    A Torso is a kinematic chain connecting the base of the robot with a collection of other kinematic chains.
    """

    def assign_to_robot(self, robot: AbstractRobot):
        """
        Assigns the torso to the given robot and calls the appropriate method for each of its attached kinematic chains.
         This method ensures that the torso is only assigned to one robot at a time, and raises an error if it is
         already assigned to another robot.
        """
        if self._robot is not None and self._robot != robot:
            raise ValueError(f"Torso {self.name} is already part of another robot: {self._robot.name}.")
        if self._robot is not None and self._robot == robot:
            return
        self._robot = robot

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tip))


@dataclass
class CollisionAvoidanceThreshold:
    hard_threshold: float = 0.0
    """
    MUST stay at least this distance away from other bodies.
    """

    soft_threshold: float = 0.0
    """
    SHOULD stay at most this distance away from other bodies.
    """

    number_of_repeller: int = 1
    """
    From how many bodies can this body be repelled from at the same time.
    """

    def __post_init__(self):
        assert self.soft_threshold >= self.hard_threshold


@dataclass
class CollisionConfig(View):
    """
    Robot-specific collision configuration
    """

    _robot: AbstractRobot
    """
    The robot this collision configuration belongs to.
    """

    default_external_threshold: CollisionAvoidanceThreshold = field(default_factory=CollisionAvoidanceThreshold)
    """
    Default thresholds that are used for collision checking, if no matches in external_avoidance_threshold found.
    """
    external_avoidance_threshold: Dict[Body, CollisionAvoidanceThreshold] = field(default_factory=dict)
    """
    Thresholds used for collision avoidance between bodies of the robot and bodies which don't belong to the robot.
    """

    default_self_threshold: CollisionAvoidanceThreshold = field(default_factory=CollisionAvoidanceThreshold)
    """
    Default thresholds that are used for collision checking, if no matches in self_avoidance_threshold found.
    """
    self_avoidance_threshold: Dict[Body, CollisionAvoidanceThreshold] = field(default_factory=dict)
    """
    Thresholds used for collision avoidance between bodies belonging the robot.
    """

    disabled_bodies: Set[Body] = field(default_factory=set)
    """
    Bodies for which collisions should never be checked with anything.
    """

    disabled_pairs: Set[Tuple[Body, Body]] = field(default_factory=set)
    """
    Pairs for bodies for which collisions should never be checked.
    """

    frozen_connections: Set[Connection] = field(default_factory=set)
    """
    Connections that should be treated as fixed for collision avoidance.
    Common example are gripper joints, you generally don't want to avoid collisions by closing the fingers, 
    but by moving the whole hand away.
    """

    SRDF_DISABLE_ALL_COLLISIONS: str = 'disable_all_collisions'
    SRDF_DISABLE_SELF_COLLISION: str = 'disable_self_collision'
    SRDF_MOVEIT_DISABLE_COLLISIONS: str = 'disable_collisions'
    """
    Constants for SRDF tags top help with parsing.
    """

    def __post_init__(self):
        self.name = PrefixedName(f'{self._robot.name} collision config')
        disabled_pairs = self.compute_uncontrolled_body_pairs()
        self.disabled_pairs.update(disabled_pairs)
        super().__post_init__()
        external_avoidance_threshold = defaultdict(lambda: self.default_external_threshold)
        external_avoidance_threshold.update(self.external_avoidance_threshold)
        self.external_avoidance_threshold = external_avoidance_threshold

        self_avoidance_threshold = defaultdict(lambda: self.default_self_threshold)
        self_avoidance_threshold.update(self.self_avoidance_threshold)
        self.self_avoidance_threshold = self_avoidance_threshold

    @staticmethod
    def sort_bodies(body_a: Body, body_b: Body) -> Tuple[Body, Body]:
        """
        Sort two bodies by their names to ensure consistent ordering and avoid duplicates in sets.
        
        :param body_a: First body to compare
        :param body_b: Second body to compare
        :return: Tuple of (body_a, body_b) sorted by their names
        """
        if body_a.name > body_b.name:
            return body_b, body_a
        return body_a, body_b

    def set_external_threshold_for_connection(self, connection: Connection, threshold: CollisionAvoidanceThreshold):
        bodies = self._robot.get_directly_child_bodies_with_collision(connection)
        for body in bodies:
            self.external_avoidance_threshold[body] = threshold

    def compute_uncontrolled_body_pairs(self) -> Set[Tuple[Body, Body]]:
        """
        Computes pairs of bodies that should not be collision checked because they have no controlled connections between them.

        When all connections between two bodies are not controlled, these bodies cannot move relative to each
        other, so collision checking between them is unnecessary.
        
        :return: Set of body pairs that should have collisions disabled
        """
        body_combinations = set(combinations_with_replacement(self._robot.bodies_with_collisions, 2))
        body_combinations = {self.sort_bodies(*x) for x in body_combinations}
        disabled_pairs = set()
        for body_a, body_b in list(body_combinations):
            body_a, body_b = self.sort_bodies(body_a, body_b)
            if body_a == body_b:
                continue
            if self._robot.is_controlled_connection_in_chain(body_a, body_b):
                continue
            disabled_pairs.add((body_a, body_b))
        return disabled_pairs

    @classmethod
    def from_srdf(cls, file_path: str, world: World, robot: AbstractRobot) -> CollisionConfig:
        """
        Creates a CollisionConfig instance from an SRDF file.

        Parse an SRDF file to configure disabled collision pairs or bodies for a given world.
        Process SRDF elements like `disable_collisions`, `disable_self_collision`,
        or `disable_all_collisions` to update collision configuration
        by referencing bodies in the provided `world`.

        :param file_path: The path to the SRDF file used for collision configuration.
        :param world: The World instance containing all the bodies and their
                      collision properties.
        :param robot: The AbstractRobot instance this collision configuration belongs to.
        :return: An instance of CollisionConfig with disabled collision pairs and
                 bodies updated based on the SRDF file.
        """
        disabled_pairs = set()
        disabled_bodies = set()

        if not os.path.exists(file_path):
            raise ValueError(f'file {file_path} does not exist')
        srdf = etree.parse(file_path)
        srdf_root = srdf.getroot()
        for child in srdf_root:
            if hasattr(child, 'tag'):
                if child.tag in {cls.SRDF_MOVEIT_DISABLE_COLLISIONS, cls.SRDF_DISABLE_SELF_COLLISION}:
                    body_a_srdf_name: str = child.attrib['link1']
                    body_b_srdf_name: str = child.attrib['link2']
                    body_a = world.get_body_by_name(body_a_srdf_name)
                    body_b = world.get_body_by_name(body_b_srdf_name)
                    if body_a not in world.bodies_with_collisions:
                        continue
                    if body_b not in world.bodies_with_collisions:
                        continue
                    body_a, body_b = cls.sort_bodies(body_a, body_b)
                    disabled_pairs.add((body_a, body_b))
                elif child.tag == cls.SRDF_DISABLE_ALL_COLLISIONS:
                    body = world.get_body_by_name(child.attrib['link'])
                    disabled_bodies.add(body)
        return cls(_world=world,
                   _robot=robot,
                   disabled_bodies=disabled_bodies,
                   disabled_pairs=disabled_pairs)

    def save_to_file(self, file_path: str):
        # Create the root element
        root = etree.Element('robot')
        root.append('name', self._robot.name)

        # %% disabled bodies
        for body in sorted(self.disabled_bodies, key=lambda body: body.name):
            child = etree.SubElement(root, self.SRDF_DISABLE_ALL_COLLISIONS)
            child.append('link', body.name.name)

        # %% disabled body pairs
        for (body_a, body_b), reason in sorted(self.disabled_pairs):
            child = etree.SubElement(root, self.SRDF_DISABLE_SELF_COLLISION)
            child.append('link1', body_a.name.name)
            child.append('link2', body_b.name.name)
            child.append('reason', reason.name)

        # Create the XML tree
        tree = etree.ElementTree(root)
        tree.write(file_path, pretty_print=True, xml_declaration=True, encoding=tree.docinfo.encoding)


@dataclass
class AbstractRobot(RootedView, ABC):
    """
    Specification of an abstract robot. A robot consists of:
    - a root body, which is the base of the robot
    - an optional torso, which is a kinematic chain (usually without a manipulator) connecting the base with a collection
        of other kinematic chains
    - an optional collection of manipulator chains, each containing a manipulator, such as a gripper
    - an optional collection of sensor chains, each containing a sensor, such as a camera
    => If a kinematic chain contains both a manipulator and a sensor, it will be part of both collections
    """
    odom: Body = field(default_factory=Body)
    """
    The odometry body of the robot, which is usually the base footprint.
    """

    drive: Optional[OmniDrive] = None
    """
    The connection which the robot uses for driving.
    """

    torso: Optional[Torso] = None
    """
    The torso of the robot, which is a kinematic chain connecting the base with a collection of other kinematic chains.
    """

    manipulators: Set[Manipulator] = field(default_factory=set)
    """
    A collection of manipulators in the robot, such as grippers.
    """

    sensors: Set[Sensor] = field(default_factory=set)
    """
    A collection of sensors in the robot, such as cameras.
    """

    manipulator_chains: Set[KinematicChain] = field(default_factory=set)
    """
    A collection of all kinematic chains containing a manipulator, such as a gripper.
    """

    sensor_chains: Set[KinematicChain] = field(default_factory=set)
    """
    A collection of all kinematic chains containing a sensor, such as a camera.
    """

    controlled_connections: Set[ActiveConnection] = field(default_factory=set)
    """
    A subset of the robot's connections that are controlled by a controller.
    """

    collision_config: Optional[CollisionConfig] = None
    """
    Robot-specific collision configuration.
    """

    def load_collision_config(self, file_path: str):
        self.collision_config = CollisionConfig.from_srdf(file_path=file_path, world=self._world, robot=self)

    @classmethod
    @abstractmethod
    def from_world(cls, world: World) -> Self:
        """
        Creates a robot view from the given world.
        This method constructs the robot view by identifying and organizing the various semantic components of the robot,
        such as manipulators, sensors, and kinematic chains. It is expected to be implemented in subclasses.

        :param world: The world from which to create the robot view.

        :return: A robot view.
        """
        raise NotImplementedError("This method should be implemented in subclasses.")

    def add_manipulator(self, manipulator: Manipulator):
        """
        Adds a manipulator to the robot's collection of manipulators.
        """
        self.manipulators.add(manipulator)
        self._views.add(manipulator)
        manipulator.assign_to_robot(self)

    def add_sensor(self, sensor: Sensor):
        """
        Adds a sensor to the robot's collection of sensors.
        """
        self.sensors.add(sensor)
        self._views.add(sensor)
        sensor.assign_to_robot(self)

    def add_torso(self, torso: Torso):
        """
        Adds a torso to the robot's collection of kinematic chains.
        """
        if self.torso is not None:
            raise ValueError(f"Robot {self.name} already has a torso: {self.torso.name}.")
        self.torso = torso
        self._views.add(torso)
        torso.assign_to_robot(self)

    def add_kinematic_chain(self, kinematic_chain: KinematicChain):
        """
        Adds a kinematic chain to the robot's collection of kinematic chains.
        This can be either a manipulator chain or a sensor chain.
        """
        if kinematic_chain.manipulator is None and not kinematic_chain.sensors:
            logging.warning(
                f"Kinematic chain {kinematic_chain.name} has no manipulator or sensors, so it was skipped. Did you mean to add it to the torso?")
            return
        if kinematic_chain.manipulator is not None:
            self.manipulator_chains.add(kinematic_chain)
        if kinematic_chain.sensors:
            self.sensor_chains.add(kinematic_chain)
        self._views.add(kinematic_chain)
        kinematic_chain.assign_to_robot(self)

    @lru_cache(maxsize=None)
    def is_controlled_connection_in_chain(self, root: Body, tip: Body) -> bool:
        root_part, tip_part = self._world.compute_split_chain_of_connections(root, tip)
        connections = root_part + tip_part
        for c in connections:
            if c in self.controlled_connections:
                return True
        return False

    @cached_property
    def unmovable_bodies_with_collision(self) -> Set[Body]:
        result = set()
        for body in self.bodies_with_collisions:
            if not self.is_controlled_connection_in_chain(self._world.root, body):
                result.add(body)
        return result


    @lru_cache(maxsize=None)
    def get_controlled_parent_connection(self, body: Body) -> Connection:
        if body == self.root:
            raise ValueError(f"Cannot get controlled parent connection for root body {self.root.name}.")
        if body.parent_connection in self.controlled_connections:
            return body.parent_connection
        return self.get_controlled_parent_connection(body.parent_body)

    @cached_property
    def bodies_with_collisions(self) -> List[Body]:
        return [x for x in self.bodies if x.has_collision()]

    def compute_chain_reduced_to_controlled_joints(self, root: Body, tip: Body) -> Tuple[Body, Body]:
        """
        Removes root and tip links until they are both connected with a controlled connection.
        Useful for implementing collision avoidance.

        1. Compute the kinematic chain of bodies between root and tip.
        2. Remove all entries from link_a downward until one is connected with a connection from this view.
        2. Remove all entries from link_b upward until one is connected with a connection from this view.

        :param root: start of the chain
        :param tip: end of the chain
        :return: start and end link of the reduced chain
        """
        downward_chain, upward_chain = self._world.compute_split_chain_of_connections(root=root, tip=tip)
        chain = downward_chain + upward_chain
        for i, connection in enumerate(chain):
            if connection in self.connections:
                new_root = connection
                break
        else:
            raise KeyError(f'no controlled connection in chain between {root} and {tip}')
        for i, connection in enumerate(reversed(chain)):
            if connection in self.connections:
                new_tip = connection
                break
        else:
            raise KeyError(f'no controlled connection in chain between {root} and {tip}')

        if new_root in upward_chain:
            new_root_body = new_root.parent
        else:  # if new_root is in the downward chain, we need to "flip" it by returning its child
            new_root_body = new_root.child
        if new_tip in upward_chain:
            new_tip_body = new_tip.child
        else:  # if new_root is in the downward chain, we need to "flip" it by returning its parent
            new_tip_body = new_tip.parent
        return new_root_body, new_tip_body

    def get_directly_child_bodies_with_collision(self, connection: Connection) -> Set[Body]:
        class BodyCollector(rx.visit.DFSVisitor):
            def __init__(self, world: World, frozen_connections: Set[Connection]):
                self.world = world
                self.bodies = set()
                self.frozen_connections = frozen_connections

            def discover_vertex(self, node_index: int, time: int) -> None:
                body = self.world.kinematic_structure[node_index]
                if body.has_collision():
                    self.bodies.add(body)

            def tree_edge(self, e):
                if e in self.frozen_connections:
                    return
                if isinstance(e, ActiveConnection):
                    raise rx.visit.PruneSearch()

        visitor = BodyCollector(self._world, self.collision_config.frozen_connections)
        rx.dfs_search(self._world.kinematic_structure, [connection.child.index], visitor)

        return visitor.bodies


@dataclass
class PR2(AbstractRobot):
    """
    Represents the Personal Robot 2 (PR2), which was originally created by Willow Garage.
    The PR2 robot consists of two arms, each with a parallel gripper, a head with a camera, and a prismatic torso
    """
    neck: Neck = field(init=False)
    left_arm: KinematicChain = field(init=False)
    right_arm: KinematicChain = field(init=False)

    def __hash__(self):
        return hash(self.name)

    def _add_arm(self, arm: KinematicChain, arm_side: str):
        """
        Adds a kinematic chain to the PR2 robot's collection of kinematic chains.
        If the kinematic chain is an arm, it will be added to the left or right arm accordingly.

        :param arm: The kinematic chain to add to the PR2 robot.
        """
        if arm.manipulator is None:
            raise ValueError(f"Arm kinematic chain {arm.name} must have a manipulator.")

        if arm_side == 'left':
            self.left_arm = arm
        elif arm_side == 'right':
            self.right_arm = arm
        else:
            raise ValueError(f"Invalid arm side: {arm_side}. Must be 'left' or 'right'.")

        super().add_kinematic_chain(arm)

    def add_left_arm(self, kinematic_chain: KinematicChain):
        """
        Adds a left arm kinematic chain to the PR2 robot.

        :param kinematic_chain: The kinematic chain representing the left arm.
        """
        self._add_arm(kinematic_chain, 'left')

    def add_right_arm(self, kinematic_chain: KinematicChain):
        """
        Adds a right arm kinematic chain to the PR2 robot.

        :param kinematic_chain: The kinematic chain representing the right arm.
        """
        self._add_arm(kinematic_chain, 'right')

    def add_neck(self, neck: Neck):
        """
        Adds a neck kinematic chain to the PR2 robot.

        :param neck: The neck kinematic chain to add.
        """
        if not neck.sensors:
            raise ValueError(f"Neck kinematic chain {neck.name} must have at least one sensor.")
        self.neck = neck
        super().add_kinematic_chain(neck)

    @classmethod
    def from_world(cls, world: World) -> Self:
        """
        Creates a PR2 robot view from the given world.

        :param world: The world from which to create the robot view.

        :return: A PR2 robot view.
        """

        robot = cls(
            name=PrefixedName(name='pr2', prefix=world.name),
            odom=world.get_body_by_name("odom_combined"),
            root=world.get_body_by_name("base_footprint"),
            _world=world,
        )

        robot.drive = robot.odom.parent_connection

        # Create left arm
        left_gripper_thumb = Finger(name=PrefixedName('left_gripper_thumb', prefix=robot.name.name),
                                    root=world.get_body_by_name("l_gripper_l_finger_link"),
                                    tip=world.get_body_by_name("l_gripper_l_finger_tip_link"),
                                    _world=world)

        left_gripper_finger = Finger(name=PrefixedName('left_gripper_finger', prefix=robot.name.name),
                                     root=world.get_body_by_name("l_gripper_r_finger_link"),
                                     tip=world.get_body_by_name("l_gripper_r_finger_tip_link"),
                                     _world=world)

        left_gripper = ParallelGripper(name=PrefixedName('left_gripper', prefix=robot.name.name),
                                       root=world.get_body_by_name("l_gripper_palm_link"),
                                       tool_frame=world.get_body_by_name("l_gripper_tool_frame"),
                                       thumb=left_gripper_thumb,
                                       finger=left_gripper_finger,
                                       _world=world)

        left_arm = Arm(name=PrefixedName('left_arm', prefix=robot.name.name),
                       root=world.get_body_by_name("l_shoulder_pan_link"),
                       manipulator=left_gripper,
                       _world=world)

        robot.add_left_arm(left_arm)

        # Create right arm
        right_gripper_thumb = Finger(name=PrefixedName('right_gripper_thumb', prefix=robot.name.name),
                                     root=world.get_body_by_name("r_gripper_l_finger_link"),
                                     tip=world.get_body_by_name("r_gripper_l_finger_tip_link"),
                                     _world=world)
        right_gripper_finger = Finger(name=PrefixedName('right_gripper_finger', prefix=robot.name.name),
                                      root=world.get_body_by_name("r_gripper_r_finger_link"),
                                      tip=world.get_body_by_name("r_gripper_r_finger_tip_link"),
                                      _world=world)
        right_gripper = ParallelGripper(name=PrefixedName('right_gripper', prefix=robot.name.name),
                                        root=world.get_body_by_name("r_gripper_palm_link"),
                                        tool_frame=world.get_body_by_name("r_gripper_tool_frame"),
                                        thumb=right_gripper_thumb,
                                        finger=right_gripper_finger,
                                        _world=world)
        right_arm = Arm(name=PrefixedName('right_arm', prefix=robot.name.name),
                        root=world.get_body_by_name("r_shoulder_pan_link"),
                        manipulator=right_gripper,
                        _world=world)

        robot.add_right_arm(right_arm)

        # Create camera and neck
        camera = Camera(name=PrefixedName('wide_stereo_optical_frame', prefix=robot.name.name),
                        root=world.get_body_by_name('wide_stereo_optical_frame'),
                        forward_facing_axis=Vector3(0, 0, 1),
                        field_of_view=FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049),
                        minimal_height=1.27,
                        maximal_height=1.60,
                        _world=world)

        neck = Neck(name=PrefixedName('neck', prefix=robot.name.name),
                    sensors={camera},
                    root=world.get_body_by_name("head_pan_link"),
                    tip=world.get_body_by_name("head_tilt_link"),
                    _world=world)
        robot.add_neck(neck)

        # Create torso
        torso = Torso(name=PrefixedName('torso', prefix=robot.name.name),
                      root=world.get_body_by_name("torso_lift_link"),
                      tip=world.get_body_by_name("torso_lift_link"),
                      _world=world)
        robot.add_torso(torso)

        world.add_view(robot)

        return robot
