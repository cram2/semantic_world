from __future__ import annotations

import logging
from abc import abstractmethod, ABC
from dataclasses import dataclass, field
from typing import Tuple, Iterable, Set

from typing_extensions import Optional, List, Self

from .prefixed_name import PrefixedName
from .spatial_types.spatial_types import Vector3
from .world import World
from .world_entity import Body, RootedView, Connection


@dataclass
class RobotView(RootedView):
    """
    Represents a collection of connected robot bodies, starting from a root body, and ending in a unspecified collection
    of tip bodies.
    """
    _robot: AbstractRobot = field(default=None)
    """
    The robot this view belongs to
    """

    @abstractmethod
    def create(cls, *args, **kwargs) -> Self:
        """
        Creates a robot view from the given arguments. This method should be implemented in subclasses to ensure
        proper creation of the robot view.

        :return: A robot view.
        """
        raise NotImplementedError("This method should be implemented in subclasses.")

    @abstractmethod
    def assign_to_robot(self, robot: AbstractRobot):
        """
        This method assigns the robot to the current view, and then iterates through its own fields to call the
        appropriate methods to att them to the robot.

        :param robot: The robot to which this view should be assigned.
        """
        raise NotImplementedError("This method should be implemented in subclasses.")


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

    @classmethod
    def create(cls, world: World, name: str, root_name: str, manipulator: Manipulator,
               sensors: Set[Sensor] = None) -> Self:
        """
        Creates a KinematicChain object representing an arm, starting from the shoulder body and ending at the gripper.

        :param world: The world from which to get the body objects.
        :param name: The name of the kinematic chain.
        :param root_name: The name of the shoulder body in the world.
        :param manipulator: An optional manipulator (e.g., a gripper) that is part of the arm.
        :param sensors: An optional Set of sensors that are part of the arm.

        :return: A KinematicChain object if the shoulder body and gripper are found, otherwise raises ValueError.
        """
        if sensors is None:
            sensors = []

        root = world.get_body_by_name(root_name)
        arm_tip = manipulator.root.parent_body

        arm = cls(
            name=PrefixedName(name=name, prefix=world.name),
            root=root,
            tip=arm_tip,
            manipulator=manipulator,
            sensors=sensors,
            _world=world,
        )

        world.add_view(arm)
        return arm

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

    @classmethod
    def create(cls, world: World, name: str, root_name: str, tip_name: str, sensors: Set[Sensor] = None) -> Self:
        """
        :param world: The world from which to get the body objects
        :param name: The name of the finger, which will be prefixed with the world name.
        :param root_name: The name of the root body in the world.
        :param tip_name: The name of the tip body in the world.
        :param sensors: An optional Set of sensors that are part of the finger.

        :return: A Finger object if the root and tip bodies are found, raise ValueError otherwise.
        """
        if sensors is None:
            sensors = []
        root = world.get_body_by_name(root_name)
        tip = world.get_body_by_name(tip_name)

        finger = cls(
            root=root,
            tip=tip,
            name=PrefixedName(name=name, prefix=world.name),
            sensors=sensors,
            _world=world,
        )
        world.add_view(finger)

        return finger

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

    @classmethod
    def create(cls, world: World, name: str, palm_body_name: str, tool_frame_name: str,
               thumb: Finger, second_finger: Finger) -> Self:
        """
        Creates a ParallelGripper object from the given palm body name, tool frame name, and finger body pairs.

        :param world: The world from which to get the body objects.
        :param name: A side to use for the name of the gripper.
        :param palm_body_name: The name of the palm body in the world.
        :param tool_frame_name: The name of the tool frame body in the world.
        :param thumb: The thumb which always needs to touch an object when grasping it.
        :param second_finger: The second finger of the gripper
        :return: A ParallelGripper object if the palm and tool frame bodies are found, otherwise raises ValueError.
        """

        palm_body = world.get_body_by_name(palm_body_name)
        tool_frame = world.get_body_by_name(tool_frame_name)

        parallel_gripper = cls(
            name=PrefixedName(name=name + '_gripper', prefix=world.name),
            root=palm_body,
            finger=second_finger,
            thumb=thumb,
            tool_frame=tool_frame,
            _world=world,
        )

        world.add_view(parallel_gripper)
        return parallel_gripper

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

    @classmethod
    def create(cls, world: World, camera_name: str, forward_facing_axis: Vector3,
               field_of_view: FieldOfView, minimal_height: float, maximal_height: float) -> Camera:
        """
        Creates a Camera object from the given camera sensor.

        :param world: The world from which to get the body objects.
        :param camera_name: The name of the camera body in the world.
        :param forward_facing_axis: The axis that the camera is facing
        :param field_of_view: The field of view of the camera, defined by vertical and horizontal angles.
        :param minimal_height: The minimal height of the camera above the ground.
        :param maximal_height: The maximal height of the camera above the ground.

        :return: A Camera object if the camera body is found, otherwise raises ValueError.
        """
        camera_body = world.get_body_by_name(camera_name)
        camera = cls(
            name=PrefixedName(name=camera_name, prefix=world.name),
            root=camera_body,
            forward_facing_axis=forward_facing_axis,
            field_of_view=field_of_view,
            minimal_height=minimal_height,
            maximal_height=maximal_height,
            _world=world,
        )

        world.add_view(camera)
        return camera

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

    @classmethod
    def create(cls, world: World, name: str, sensors: Set[Sensor], root_name: str, tip_name: str) -> Neck:
        """
        Creates a Neck object from the given camera sensor.

        :param world: The world from which to get the body objects.
        :param name: The name of the neck, which will be prefixed with the world name.
        :param sensors: A Set of sensors that are part of the neck, such as cameras.
        :param root_name: The name of the root body of the neck
        :param tip_name: The name of the tip body of the neck

        :return: A Neck object if the camera is found, otherwise raises ValueError.
        """
        if not sensors:
            raise ValueError("Neck must have at least one sensor")

        root = world.get_body_by_name(root_name)
        tip = world.get_body_by_name(tip_name)

        neck = cls(
            name=PrefixedName(name=name, prefix=world.name),
            root=root,
            tip=tip,
            sensors=sensors,
            _world=world,
        )

        world.add_view(neck)
        return neck

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
    kinematic_chains: Set[KinematicChain] = field(default_factory=set)
    """
    A collection of kinematic chains, such as sensor chains or manipulation chains, that are connected to the torso.
    """

    @classmethod
    def create(cls, world: World, name: str, root_name: str, tip_name: str,
               manipulator_chains: Set[KinematicChain], sensor_chains: Set[KinematicChain]) -> Torso:
        """
        Creates a Torso object from the given manipulator and sensor chains.

        :param world: The world from which to get the body objects.
        :param name: The name of the torso, which will be prefixed with the world name.
        :param root_name: The name of the root body of the torso in the world.
        :param tip_name: The name of the tip body of the torso in the world.
        :param manipulator_chains: A Set of KinematicChain objects representing the manipulators of the robot.
        :param sensor_chains: A Set of KinematicChain objects representing the sensors of the robot.

        :return: A Torso object if the torso body is found, otherwise raises ValueError.
        """
        torso_root = world.get_body_by_name(root_name)
        torso_tip = world.get_body_by_name(tip_name)

        torso = cls(
            name=PrefixedName(name=name, prefix=world.name),
            root=torso_root,
            tip=torso_tip,
            kinematic_chains=manipulator_chains | sensor_chains,
            _world=world,
        )
        world.add_view(torso)
        return torso

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
        for chain in self.kinematic_chains:
            robot.add_kinematic_chain(chain)

    def __hash__(self):
        """
        Returns the hash of the kinematic chain, which is based on the root and tip bodies.
        This allows for proper comparison and storage in sets or dictionaries.
        """
        return hash((self.name, self.root, self.tip))

@dataclass
class AbstractRobot(RootedView):
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


@dataclass
class PR2(AbstractRobot):
    """
    Represents the Personal Robot 2 (PR2), which was originally created by Willow Garage.
    The PR2 robot consists of two arms, each with a parallel gripper, a head with a camera, and a prismatic torso
    """
    neck: Neck = field(default_factory=Neck)
    left_arm: KinematicChain = field(default_factory=KinematicChain)
    right_arm: KinematicChain = field(default_factory=KinematicChain)

    def add_kinematic_chain(self, kinematic_chain: KinematicChain):
        """
        Adds a kinematic chain to the PR2 robot's collection of kinematic chains.
        If the kinematic chain is an arm, it will be added to the left or right arm accordingly.

        :param kinematic_chain: The kinematic chain to add to the PR2 robot.
        """
        if isinstance(kinematic_chain, Arm):
            if  kinematic_chain.name.name.startswith("left"):
                self.left_arm = kinematic_chain
            elif kinematic_chain.name.name.startswith("right"):
                self.right_arm = kinematic_chain
            else:
                logging.warning(f"Kinematic chain {kinematic_chain.name} is not recognized as a left or right arm.")
        elif isinstance(kinematic_chain, Neck):
            self.neck = kinematic_chain
        super().add_kinematic_chain(kinematic_chain)


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

        # Create left arm
        left_gripper_thumb = Finger.create(world, 'left_gripper_thumb', "l_gripper_l_finger_link", "l_gripper_l_finger_tip_link")
        left_gripper_finger = Finger.create(world, 'left_gripper_finger', "l_gripper_r_finger_link", "l_gripper_r_finger_tip_link")

        left_gripper = ParallelGripper.create(world, "left_gripper", "l_gripper_palm_link",
                                              "l_gripper_tool_frame", left_gripper_thumb, left_gripper_finger)

        left_arm = Arm.create(world, "left_arm", "l_shoulder_pan_link", left_gripper)
        robot.add_kinematic_chain(left_arm)

        # Create right arm
        right_gripper_thumb = Finger.create(world, 'right_gripper_thumb', "r_gripper_l_finger_link", "r_gripper_l_finger_tip_link")
        right_gripper_finger = Finger.create(world, 'right_gripper_finger', "r_gripper_r_finger_link", "r_gripper_r_finger_tip_link")
        right_gripper = ParallelGripper.create(world, "right_gripper", "r_gripper_palm_link", "r_gripper_tool_frame",
                                                       right_gripper_thumb, right_gripper_finger)
        right_arm = Arm.create(world, "right_arm", "r_shoulder_pan_link", right_gripper)
        robot.add_kinematic_chain(right_arm)

        # Create camera and neck
        camera = Camera.create(world, "wide_stereo_optical_frame", Vector3.from_xyz(0, 0, 1),
                               FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049), 1.27, 1.60)
        neck = Neck.create(world, "neck", sensors={camera}, root_name="head_pan_link", tip_name="head_tilt_link")
        robot.add_kinematic_chain(neck)

        # Create torso
        torso = Torso.create(world, "torso", "torso_lift_link", "torso_lift_link",
                             robot.manipulator_chains, robot.sensor_chains)
        robot.add_torso(torso)

        world.add_view(robot)

        return robot


