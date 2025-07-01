from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing_extensions import Optional, List, Self

from .world_entity import Body, RootedView


@dataclass
class RobotBody(Body):
    """
    Represents a body in a robot.
    """
    _robot: AbstractRobot = field(default=None, init=False)

    def __hash__(self):
        return super().__hash__()

@dataclass
class RobotView(RootedView):
    """
    Represents a collection of connected robot bodies, starting from a root body, and ending in a unspecified collection
    of tip bodies.
    """
    _robot: AbstractRobot = field(default=None, init=False)
    identifier: str = field(default_factory=str)


@dataclass
class KinematicChain(RobotView):
    """
    Represents a kinematic chain in a robot, starting from a root body, and ending in a specific tip body.
    A kinematic chain can contain both manipulators and sensors at the same time, and is not limited to a single
    instance of each.
    """
    tip_body: RobotBody = field(default_factory=RobotBody)
    manipulator: Optional[Manipulator] = None
    sensors: Optional[List[Sensor]] = None

class Direction(int, enum.Enum):
    POSITIVE = 1
    NEGATIVE = -1

class AxisIdentifier(enum.Enum):
    """
    Enum for translating the axis name to a vector along that axis.
    """
    X = (1, 0, 0)
    Y = (0, 1, 0)
    Z = (0, 0, 1)


@dataclass
class AxisDirection:
    axis: AxisIdentifier
    direction: Direction


@dataclass
class Manipulator(RobotView):
    """
    Represents a manipulator of a robot. Always has a tool frame.
    """
    tool_frame: RobotBody = field(default_factory=RobotBody)


@dataclass
class Finger(KinematicChain):
    """
    A finger is a kinematic chain, since it should have an unambiguous tip body, and may contain sensors.
    """
    ...


@dataclass
class Gripper(Manipulator):
    """
    Represents a gripper of a robot. Contains a collection of fingers and a thumb. The thumb is a specific finger
    that always needs to touch an object when grasping it, ensuring a stable grasp.
    """
    fingers: List[Finger] = field(default_factory=list)
    thumb: Optional[Finger] = None


@dataclass
class Sensor(RobotBody):
    """
    Represents any kind of sensor in a robot.
    """
    identifier: str = field(default_factory=str)


@dataclass
class FieldOfView:
    vertical_angle: float
    horizontal_angle: float


@dataclass
class Camera(Sensor):
    """
    Represents a camera sensor in a robot.
    """
    forward_facing_axis: AxisDirection = field(default_factory=AxisDirection)
    field_of_view: FieldOfView = field(default_factory=FieldOfView)
    minimal_height: float = 0.0
    maximal_height: float = 1.0


@dataclass
class Neck(KinematicChain):
    """
    Represents a special kinematic chain to identify the different bodys of the neck, which is useful to calculate
    for example "LookAt" joint states without needing IK
    """
    roll_body: Optional[RobotBody] = None
    pitch_body: Optional[RobotBody] = None
    yaw_body: Optional[RobotBody] = None


@dataclass
class Torso(KinematicChain):
    """
    A Torso is a kinematic chain connecting the base of the robot with a collection of other kinematic chains.
    """
    kinematic_chains: List[KinematicChain] = field(default_factory=list)


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
    odom: RobotBody = field(default_factory=RobotBody)
    """
    The odometry body of the robot, which is usually the base footprint.
    """

    torso: Optional[Torso] = None
    """
    The torso of the robot, which is a kinematic chain connecting the base with a collection of other kinematic chains.
    """

    manipulators: List[Manipulator] = field(default_factory=list)
    """
    A collection of manipulators in the robot, such as grippers.
    """

    sensors: List[Sensor] = field(default_factory=list)
    """
    A collection of sensors in the robot, such as cameras.
    """

    manipulator_chains: List[KinematicChain] = field(default_factory=list)
    """
    A collection of all kinematic chains containing a manipulator, such as a gripper.
    """

    sensor_chains: List[KinematicChain] = field(default_factory=list)
    """
    A collection of all kinematic chains containing a sensor, such as a camera.
    """

    def __repr__(self):
        manipulator_identifiers = [chain.identifier for chain in self.manipulator_chains] if self.manipulator_chains else []
        sensor_identifiers = [chain.identifier for chain in self.sensor_chains] if self.sensor_chains else []
        return f"<{self.__class__.__name__} base={self.root.name}, torso={self.torso.identifier}, manipulators={manipulator_identifiers}, sensors={sensor_identifiers}>"

    def __str__(self):
        def format_tree(node, prefix="", is_last=True):
            label, children = node
            branch = "└── " if is_last else "├── "
            result = prefix + branch + label
            if children:
                new_prefix = prefix + ("    " if is_last else "│   ")
                for idx, child in enumerate(children):
                    result += "\n" + format_tree(child, new_prefix, idx == len(children) - 1)
            return result

        def make_base_node(base):
            base_label = f"root ({base.__class__.__name__}): {base.name}"
            return base_label, []

        def make_drive_node(base):
            base_label = f"drive ({base.parent_connection.__class__.__name__}): {base.parent_connection.name}"
            return base_label, []

        def make_chain_node(chain: KinematicChain):
            children = []
            if chain.manipulator:
                manip_label = f"{chain.manipulator.identifier} ({chain.manipulator.__class__.__name__}):"
                manip_children = [(f"Tool Frame: {chain.manipulator.tool_frame.name}", [])]
                if isinstance(chain.manipulator, Gripper):
                    if chain.manipulator.fingers:
                        finger_children = []
                        for idx, finger in enumerate(chain.manipulator.fingers):
                            finger_children.append((
                                f"[{idx}] {finger.identifier} ({finger.__class__.__name__}): {finger.root.name} → {finger.tip_body.name}", []
                            ))
                        if chain.manipulator.thumb:
                            finger_children.append((
                                f"{chain.manipulator.thumb.identifier} ({chain.manipulator.thumb.__class__.__name__}): {chain.manipulator.thumb.root.name} → {chain.manipulator.thumb.tip_body.name}",
                                []
                            ))
                        manip_children.append(("Fingers:", finger_children))
                children.append((manip_label, manip_children))
            if chain.sensors:
                sensor_children = []
                for sensor in chain.sensors:
                    if isinstance(sensor, Camera):
                        sensor_children.append((
                            f"{sensor.identifier} ({sensor.__class__.__name__}): [Camera, Axis={sensor.forward_facing_axis.axis.name}, Direction={sensor.forward_facing_axis.direction.name}, "
                            f"FOV=({sensor.field_of_view.horizontal_angle:.2f}, {sensor.field_of_view.vertical_angle:.2f})]",
                            []
                        ))
                    else:
                        sensor_children.append((sensor.identifier, []))
                children.append(("Sensors:", sensor_children))
            if hasattr(chain, "roll_body") and chain.roll_body:
                children.append((f"Roll body: {chain.roll_body.name}", []))
            if hasattr(chain, "pitch_body") and chain.pitch_body:
                children.append((f"Pitch body: {chain.pitch_body.name}", []))
            if hasattr(chain, "yaw_body") and chain.yaw_body:
                children.append((f"Yaw body: {chain.yaw_body.name}", []))
            return f"{chain.identifier} ({chain.__class__.__name__}): {chain.root.name} → {chain.tip_body.name}", children

        def make_torso_node(torso: Torso):
            torso_label = f"{torso.identifier} ({torso.__class__.__name__}): {torso.root.name} → {torso.tip_body.name}"
            children = [make_chain_node(kc) for kc in torso.kinematic_chains]
            return torso_label, children

        def make_manipulator_chains_node(chains: List[KinematicChain]):
            label = "Manipulator Chains:"
            children = [make_chain_node(kc) for kc in chains]
            return label, children

        def make_sensor_chains_node(chains: List[KinematicChain]):
            label = "Sensor Chains:"
            children = [make_chain_node(kc) for kc in chains]
            return label, children

        root_children = [make_base_node(self.root)]
        root_children.append(make_drive_node(self.root))
        if self.torso:
            root_children.append(make_torso_node(self.torso))
        if self.manipulator_chains:
            root_children.append(make_manipulator_chains_node(self.manipulator_chains))
        if self.sensor_chains:
            root_children.append(make_sensor_chains_node(self.sensor_chains))

        tree = (f"<{self.__class__.__name__}>", root_children)
        return "\n" + format_tree(tree, prefix="", is_last=True)


@dataclass(repr=False)
class PR2(AbstractRobot):

    @classmethod
    def get_view(cls, world) -> Self:
        """
        Creates a PR2 robot view from the given world.
        This method constructs the robot view by identifying and organizing the various components of the PR2 robot,
        including arms, grippers, fingers, sensors, and the torso.

        :param world: The world from which to create the robot view.

        :return: A PR2 robot view.
        """

        def create_fingers(world, finger_body_pairs, prefix):
            """
            Creates a list of Finger objects from the given finger body pairs.
            Current assumes the last finger in the list is the thumb, in reality not always the case

            :param world: The world from which to get the body objects.
            :param finger_body_pairs: A list of tuples containing the root and tip body names for each finger.
            :param prefix: A prefix to use for the identifiers of the fingers.

            :return: A tuple containing a list of Finger objects and the thumb Finger object.
            """
            fingers = []
            for index, (root_name, tip_name) in enumerate(finger_body_pairs):
                root_obj = world.get_body_by_name(root_name)
                tip_body_obj = world.get_body_by_name(tip_name)
                if root_obj and tip_body_obj:
                    finger = Finger(
                        root=RobotBody.from_body(root_obj),
                        tip_body=RobotBody.from_body(tip_body_obj),
                        identifier=f"{prefix}_finger_{index}",
                        _world=world,
                    )
                    fingers.append(finger)
            thumb = fingers[-1] if fingers else None
            if thumb:
                thumb.identifier = f"{prefix}_thumb"
            return fingers, thumb

        def create_gripper(world, palm_body_name, tool_frame_name, finger_bodys, prefix):
            """
            Creates a Gripper object from the given palm body name, tool frame name, and finger body pairs.
            :param world: The world from which to get the body objects.
            :param palm_body_name: The name of the palm body in the world.
            :param tool_frame_name: The name of the tool frame body in the world.
            :param finger_bodys: A list of tuples containing the root and tip body names for each finger.
            :param prefix: A prefix to use for the identifier of the gripper.
            :return: A Gripper object if the palm and tool frame bodies are found, otherwise None.
            """
            fingers, thumb = create_fingers(world, finger_bodys, prefix)
            palm_body = world.get_body_by_name(palm_body_name)
            tool_frame_body = world.get_body_by_name(tool_frame_name)
            if palm_body and tool_frame_body and thumb:
                return Gripper(
                    identifier=f"{prefix}_gripper",
                    root=RobotBody.from_body(palm_body),
                    fingers=fingers,
                    thumb=thumb,
                    tool_frame=RobotBody.from_body(tool_frame_body),
                    _world=world,
                )
            return None

        def create_arm(world, shoulder_body_name, gripper, prefix):
            """
            Creates a KinematicChain object representing an arm, starting from the shoulder body and ending at the gripper.
            :param world: The world from which to get the body objects.
            :param shoulder_body_name: The name of the shoulder body in the world.
            :param gripper: The Gripper object representing the gripper of the arm.
            :param prefix: A prefix to use for the identifier of the arm.
            :return: A KinematicChain object if the shoulder body and gripper are found, otherwise None.
            """
            shoulder_body = world.get_body_by_name(shoulder_body_name)
            if shoulder_body and gripper:
                arm_tip_body = gripper.root.parent_body
                return KinematicChain(
                    identifier=f"{prefix}_arm",
                    root=RobotBody.from_body(shoulder_body),
                    tip_body=RobotBody.from_body(arm_tip_body),
                    manipulator=gripper,
                    _world=world,
                )
            return None

        ################################# Create robot #################################
        manipulators = []
        manipulator_chains = []
        sensors = []
        sensor_chains = []
        ################################### Left Arm ###################################
        left_finger_bodys = [
            ("l_gripper_l_finger_link", "l_gripper_l_finger_tip_link"),
            ("l_gripper_r_finger_link", "l_gripper_r_finger_tip_link"),
        ]
        left_gripper = create_gripper(world, "l_gripper_palm_link", "l_gripper_tool_frame",
                                      left_finger_bodys, "left")
        manipulators.append(left_gripper)
        left_arm = create_arm(world, "l_shoulder_pan_link", left_gripper, "left")
        manipulator_chains.append(left_arm)
        ################################### Right Arm ###################################
        right_finger_bodys = [
            ("r_gripper_l_finger_link", "r_gripper_l_finger_tip_link"),
            ("r_gripper_r_finger_link", "r_gripper_r_finger_tip_link"),
        ]
        right_gripper = create_gripper(world, "r_gripper_palm_link", "r_gripper_tool_frame",
                                       right_finger_bodys, "right")
        manipulators.append(right_gripper)
        right_arm = create_arm(world, "r_shoulder_pan_link", right_gripper, "right")
        manipulator_chains.append(right_arm)

        ################################# Create camera #################################
        camera_body = world.get_body_by_name("wide_stereo_optical_frame")
        camera = Camera(
            camera_body.name,
            camera_body.visual,
            camera_body.collision,
            identifier="kinect_camera",
            forward_facing_axis=AxisDirection(AxisIdentifier.Z, Direction.POSITIVE),
            field_of_view=FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049),
            minimal_height=1.27,
            maximal_height=1.60,
            _world=world,
        ) if camera_body else None
        sensors.append(camera)

        ################################## Create head ##################################
        neck_root = world.get_body_by_name("head_pan_link")
        neck_tip_body = world.get_body_by_name("head_tilt_link")
        head = None
        if neck_root and neck_tip_body and camera:
            head = Neck(
                identifier="neck",
                root=RobotBody.from_body(neck_root),
                tip_body=RobotBody.from_body(neck_tip_body),
                sensors=[camera],
                pitch_body=neck_tip_body,
                yaw_body=neck_root,
                _world=world,
            )
            sensor_chains.append(head)

        ################################## Create torso ##################################
        torso_body = world.get_body_by_name("torso_lift_link")
        torso_root = RobotBody.from_body(torso_body) if torso_body else None
        torso = None
        if torso_root and torso_body:
            torso = Torso(
                identifier="torso",
                root=torso_root,
                tip_body=torso_root,
                kinematic_chains=[kc for kc in [left_arm, right_arm, head] if kc],
                _world=world,
            )

        ################################## Create base ##################################
        base_body = world.get_body_by_name("base_footprint")
        base_root = RobotBody.from_body(base_body) if base_body else None
        odom=world.get_body_by_name("odom_combined")
        odom_root = RobotBody.from_body(odom) if odom else None

        ################################## Create robot ##################################
        return cls(odom=odom_root, root=base_root, torso=torso, manipulators=manipulators,
                    manipulator_chains=manipulator_chains, sensors=sensors, sensor_chains=sensor_chains, _world=world)