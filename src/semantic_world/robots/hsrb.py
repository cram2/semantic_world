from dataclasses import dataclass
from typing import Self

from .robot import (
    AbstractRobot,
    Arm,
    Neck,
    Finger,
    ParallelGripper,
    Camera,
    Torso,
    FieldOfView,
)
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types.spatial_types import Vector3
from ..world import World


@dataclass
class HumanSupportRobotB(AbstractRobot):
    """
    Class that describes the Human Support Robot variant B (https://upmroboticclub.wordpress.com/robot/).
    """

    arm: Arm = None
    neck: Neck = None

    head_center_camera: Camera = None
    head_right_camera: Camera = None
    head_left_camera: Camera = None
    head_rgbd_camera: Camera = None
    hand_camera: Camera = None

    @classmethod
    def from_world(cls, world: World) -> Self:
        """
        Creates an HSRB (Human Support Robot B) view from a World that was parsed from
        resources/urdf/robots/hsrb.urdf. Assumes all URDF link names exist in the world.
        """
        robot = cls(
            name=PrefixedName("hsrb", prefix=world.name),
            root=world.get_body_by_name("base_footprint"),
            _world=world,
        )

        gripper_thumb = Finger(
            name=PrefixedName("thumb", prefix=robot.name.name),
            root=world.get_body_by_name("hand_l_distal_link"),
            tip=world.get_body_by_name("hand_l_finger_tip_frame"),
            _world=world,
        )

        gripper_finger = Finger(
            name=PrefixedName("finger", prefix=robot.name.name),
            root=world.get_body_by_name("hand_r_distal_link"),
            tip=world.get_body_by_name("hand_r_finger_tip_frame"),
        )

        gripper = ParallelGripper(
            name=PrefixedName("gripper", prefix=robot.name.name),
            root=world.get_body_by_name("hand_palm_link"),
            tool_frame=world.get_body_by_name("gripper_tool_frame"),
            thumb=gripper_thumb,
            finger=gripper_finger,
            _world=world,
        )

        arm = Arm(
            name=PrefixedName("arm", prefix=robot.name.name),
            root=world.get_body_by_name("torso_lift_link"),
            tip=world.get_body_by_name("wrist_roll_link"),
            manipulator=gripper,
            _world=world,
        )
        robot.add_kinematic_chain(arm)
        robot.arm = arm

        # Create camera and neck
        head_center_camera = Camera(
            name=PrefixedName("head_center_camera", prefix=robot.name.name),
            root=world.get_body_by_name("head_center_camera_frame"),
            forward_facing_axis=Vector3(1, 0, 0),
            field_of_view=FieldOfView(horizontal_angle=2.0, vertical_angle=2.0),
            minimal_height=0.75049,
            maximal_height=0.99483,
            _world=world,
        )
        robot.head_center_camera = head_center_camera

        # Create torso
        torso = Torso(
            name=PrefixedName("torso", prefix=robot.name.name),
            root=world.get_body_by_name("base_link"),
            tip=world.get_body_by_name("torso_lift_link"),
            _world=world,
        )
        robot.add_torso(torso)

        world.add_view(robot, exists_ok=True)

        return robot
