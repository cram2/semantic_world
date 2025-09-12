from __future__ import annotations

import logging
import os
from abc import abstractmethod, ABC
from dataclasses import dataclass, field
from typing_extensions import Iterable, Set, TYPE_CHECKING, Optional, Self

from .world_description.connections import ActiveConnection, OmniDrive
from .datastructures.prefixed_name import PrefixedName
from .spatial_types.spatial_types import Vector3, Quaternion
from .world_description.world_entity import (
    Body,
    RootedView,
    Connection,
    CollisionCheckingConfig,
)
from .world_description.world_entity import KinematicStructureEntity, Region

if TYPE_CHECKING:
    from .world import World


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
            self._world.add_view(self, exists_ok=True)

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

    tip: Body = field(default=None)
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
        Returns itself as a kinematic chain of bodies.
        """
        return [
            entity
            for entity in self._world.compute_chain_of_kinematic_structure_entities(
                self.root, self.tip
            )
            if isinstance(entity, Body)
        ]

    @property
    def kinematic_structure_entities(self) -> Iterable[KinematicStructureEntity]:
        """
        Returns itself as a kinematic chain of KinematicStructureEntity.
        """
        return self._world.compute_chain_of_kinematic_structure_entities(
            self.root, self.tip
        )

    @property
    def regions(self) -> Iterable[Region]:
        """
        Returns itself as a kinematic chain of KinematicStructureEntity.
        """
        return [
            entity
            for entity in self._world.compute_chain_of_kinematic_structure_entities(
                self.root, self.tip
            )
            if isinstance(entity, Region)
        ]

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
            raise ValueError(
                f"Kinematic chain {self.name} is already part of another robot: {self._robot.name}."
            )
        if self._robot is not None:
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

    tool_frame: Body = field(default=None)

    front_facing_orientation: Quaternion = field(default=None)
    """
    The orientation of the manipulator's tool frame, which is usually the front-facing orientation.
    """

    front_facing_axis: Vector3 = field(default=None)
    """
    The axis of the manipulator's tool frame that is facing forward.
    """


    def assign_to_robot(self, robot: AbstractRobot):
        """
        Assigns the manipulator to the given robot. This method ensures that the manipulator is only assigned
        to one robot at a time, and raises an error if it is already assigned to another robot.
        """
        if self._robot is not None and self._robot != robot:
            raise ValueError(
                f"Manipulator {self.name} is already part of another robot: {self._robot.name}."
            )
        if self._robot is not None:
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

    finger: Finger = field(default=None)
    thumb: Finger = field(default=None)

    def assign_to_robot(self, robot: AbstractRobot):
        """
        Assigns the parallel gripper to the given robot and calls the appropriate methods for the its finger and thumb.
         This method ensures that the parallel gripper is only assigned to one robot at a time, and raises an error if
         it is already assigned to another
        """
        if self._robot is not None and self._robot != robot:
            raise ValueError(
                f"ParallelGripper {self.name} is already part of another robot: {self._robot.name}."
            )
        if self._robot is not None:
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
            raise ValueError(
                f"Sensor {self.name} is already part of another robot: {self._robot.name}."
            )
        if self._robot is not None:
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

    forward_facing_axis: Vector3 = field(default=None)
    field_of_view: FieldOfView = field(default=None)
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

    pitch_body: Optional[Body] = None
    """
    The body that allows pitch movement in the neck, if it exists.
    """
    yaw_body: Optional[Body] = None
    """
    The body that allows yaw movement in the neck, if it exists.
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
            raise ValueError(
                f"Torso {self.name} is already part of another robot: {self._robot.name}."
            )
        if self._robot is not None:
            return
        self._robot = robot

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tip))


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

    default_collision_config: CollisionCheckingConfig = field(
        kw_only=True,
        default_factory=lambda: CollisionCheckingConfig(buffer_zone_distance=0.05),
    )

    @abstractmethod
    def load_srdf(self):
        """
        Loads the SRDF file for the robot, if it exists. This method is expected to be implemented in subclasses.
        """
        ...

    @property
    def controlled_connections(self) -> Set[ActiveConnection]:
        """
        A subset of the robot's connections that are controlled by a controller.
        """
        return self._world.controlled_connections & set(self.connections)

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

    @property
    def drive(self) -> Optional[OmniDrive]:
        """
        The connection which the robot uses for driving.
        """
        try:
            parent_connection = self.root.parent_connection
            if isinstance(parent_connection, OmniDrive):
                return parent_connection
        except AttributeError:
            pass

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
            raise ValueError(
                f"Robot {self.name} already has a torso: {self.torso.name}."
            )
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
                f"Kinematic chain {kinematic_chain.name} has no manipulator or sensors, so it was skipped. Did you mean to add it to the torso?"
            )
            return
        if kinematic_chain.manipulator is not None:
            self.manipulator_chains.add(kinematic_chain)
        if kinematic_chain.sensors:
            self.sensor_chains.add(kinematic_chain)
        self._views.add(kinematic_chain)
        kinematic_chain.assign_to_robot(self)


@dataclass
class PR2(AbstractRobot):
    """
    Represents the Personal Robot 2 (PR2), which was originally created by Willow Garage.
    The PR2 robot consists of two arms, each with a parallel gripper, a head with a camera, and a prismatic torso
    """

    neck: Neck = field(default=None)
    left_arm: KinematicChain = field(default=None)
    right_arm: KinematicChain = field(default=None)

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

        if arm_side == "left":
            self.left_arm = arm
        elif arm_side == "right":
            self.right_arm = arm
        else:
            raise ValueError(
                f"Invalid arm side: {arm_side}. Must be 'left' or 'right'."
            )

        super().add_kinematic_chain(arm)

    def add_left_arm(self, kinematic_chain: KinematicChain):
        """
        Adds a left arm kinematic chain to the PR2 robot.

        :param kinematic_chain: The kinematic chain representing the left arm.
        """
        self._add_arm(kinematic_chain, "left")

    def add_right_arm(self, kinematic_chain: KinematicChain):
        """
        Adds a right arm kinematic chain to the PR2 robot.

        :param kinematic_chain: The kinematic chain representing the right arm.
        """
        self._add_arm(kinematic_chain, "right")

    def add_neck(self, neck: Neck):
        """
        Adds a neck kinematic chain to the PR2 robot.

        :param neck: The neck kinematic chain to add.
        """
        if not neck.sensors:
            raise ValueError(
                f"Neck kinematic chain {neck.name} must have at least one sensor."
            )
        self.neck = neck
        super().add_kinematic_chain(neck)

    def load_srdf(self):
        """
        Loads the SRDF file for the PR2 robot, if it exists.
        """
        srdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "collision_configs", "pr2.srdf")
        self._world.load_collision_srdf(srdf_path)

    @classmethod
    def from_world(cls, world: World) -> Self:
        """
        Creates a PR2 robot view from the given world.

        :param world: The world from which to create the robot view.

        :return: A PR2 robot view.
        """

        robot = cls(
            name=PrefixedName(name="pr2", prefix=world.name),
            root=world.get_kinematic_structure_entity_by_name("base_footprint"),
            _world=world,
        )

        # Create left arm
        left_gripper_thumb = Finger(
            name=PrefixedName("left_gripper_thumb", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name(
                "l_gripper_l_finger_link"
            ),
            tip=world.get_kinematic_structure_entity_by_name(
                "l_gripper_l_finger_tip_link"
            ),
            _world=world,
        )

        left_gripper_finger = Finger(
            name=PrefixedName("left_gripper_finger", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name(
                "l_gripper_r_finger_link"
            ),
            tip=world.get_kinematic_structure_entity_by_name(
                "l_gripper_r_finger_tip_link"
            ),
            _world=world,
        )

        left_gripper = ParallelGripper(name=PrefixedName('left_gripper', prefix=robot.name.name),
                                       root=world.get_kinematic_structure_entity_by_name("l_gripper_palm_link"),
                                       tool_frame=world.get_kinematic_structure_entity_by_name("l_gripper_tool_frame"),
                                       front_facing_orientation=Quaternion(0, 0, 0, 1),
                                       front_facing_axis=Vector3(1, 0, 0),
                                       thumb=left_gripper_thumb,
                                       finger=left_gripper_finger,
                                       _world=world)
        left_arm = Arm(
            name=PrefixedName("left_arm", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name("torso_lift_link"),
            tip=world.get_kinematic_structure_entity_by_name("l_wrist_roll_link"),
            manipulator=left_gripper,
            _world=world,
        )

        robot.add_left_arm(left_arm)

        # Create right arm
        right_gripper_thumb = Finger(
            name=PrefixedName("right_gripper_thumb", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name(
                "r_gripper_l_finger_link"
            ),
            tip=world.get_kinematic_structure_entity_by_name(
                "r_gripper_l_finger_tip_link"
            ),
            _world=world,
        )
        right_gripper_finger = Finger(
            name=PrefixedName("right_gripper_finger", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name(
                "r_gripper_r_finger_link"
            ),
            tip=world.get_kinematic_structure_entity_by_name(
                "r_gripper_r_finger_tip_link"
            ),
            _world=world,
        )
        right_gripper = ParallelGripper(
            name=PrefixedName("right_gripper", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name("r_gripper_palm_link"),
            tool_frame=world.get_kinematic_structure_entity_by_name(
                "r_gripper_tool_frame"
            ),
            front_facing_orientation=Quaternion(0, 0, 0, 1),
            front_facing_axis=Vector3(1, 0, 0),
            thumb=right_gripper_thumb,
            finger=right_gripper_finger,
            _world=world,
        )
        right_arm = Arm(
            name=PrefixedName("right_arm", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name("torso_lift_link"),
            tip=world.get_kinematic_structure_entity_by_name("r_wrist_roll_link"),
            manipulator=right_gripper,
            _world=world,
        )

        robot.add_right_arm(right_arm)

        # Create camera and neck
        camera = Camera(
            name=PrefixedName("wide_stereo_optical_frame", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name(
                "wide_stereo_optical_frame"
            ),
            forward_facing_axis=Vector3(0, 0, 1),
            field_of_view=FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049),
            minimal_height=1.27,
            maximal_height=1.60,
            _world=world,
        )

        neck = Neck(
            name=PrefixedName("neck", prefix=robot.name.name),
            sensors={camera},
            root=world.get_kinematic_structure_entity_by_name("head_pan_link"),
            tip=world.get_kinematic_structure_entity_by_name("head_tilt_link"),
            pitch_body=world.get_kinematic_structure_entity_by_name("head_tilt_link"),
            yaw_body=world.get_kinematic_structure_entity_by_name("head_pan_link"),
            _world=world,
        )
        robot.add_neck(neck)

        # Create torso
        torso = Torso(
            name=PrefixedName("torso", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name("torso_lift_link"),
            tip=world.get_kinematic_structure_entity_by_name("torso_lift_link"),
            _world=world,
        )
        robot.add_torso(torso)
        # robot.load_srdf()

        world.add_view(robot, exists_ok=True)

        return robot
