from __future__ import annotations

from dataclasses import dataclass, field
from typing import Self

from ..datastructures.prefixed_name import PrefixedName
from .robot import (
    AbstractRobot,
    Neck,
    KinematicChain,
    Finger,
    ParallelGripper,
    Arm,
    Camera,
    FieldOfView,
    Torso,
)
from semantic_world.spatial_types import Vector3
from semantic_world.world import World


@dataclass
class PR2(AbstractRobot):
    """
    Represents the Personal Robot 2 (PR2), which was originally created by Willow Garage.
    The PR2 robots consists of two arms, each with a parallel gripper, a head with a camera, and a prismatic torso
    """

    neck: Neck = field(default=None)
    left_arm: KinematicChain = field(default=None)
    right_arm: KinematicChain = field(default=None)

    def __hash__(self):
        return hash(self.name)

    def _add_arm(self, arm: KinematicChain, arm_side: str):
        """
        Adds a kinematic chain to the PR2 robots's collection of kinematic chains.
        If the kinematic chain is an arm, it will be added to the left or right arm accordingly.

        :param arm: The kinematic chain to add to the PR2 robots.
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
        Adds a left arm kinematic chain to the PR2 robots.

        :param kinematic_chain: The kinematic chain representing the left arm.
        """
        self._add_arm(kinematic_chain, "left")

    def add_right_arm(self, kinematic_chain: KinematicChain):
        """
        Adds a right arm kinematic chain to the PR2 robots.

        :param kinematic_chain: The kinematic chain representing the right arm.
        """
        self._add_arm(kinematic_chain, "right")

    def add_neck(self, neck: Neck):
        """
        Adds a neck kinematic chain to the PR2 robots.

        :param neck: The neck kinematic chain to add.
        """
        if not neck.sensors:
            raise ValueError(
                f"Neck kinematic chain {neck.name} must have at least one sensor."
            )
        self.neck = neck
        super().add_kinematic_chain(neck)

    @classmethod
    def from_world(cls, world: World) -> Self:
        """
        Creates a PR2 robots view from the given world.

        :param world: The world from which to create the robots view.

        :return: A PR2 robots view.
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

        left_gripper = ParallelGripper(
            name=PrefixedName("left_gripper", prefix=robot.name.name),
            root=world.get_kinematic_structure_entity_by_name("l_gripper_palm_link"),
            tool_frame=world.get_kinematic_structure_entity_by_name(
                "l_gripper_tool_frame"
            ),
            thumb=left_gripper_thumb,
            finger=left_gripper_finger,
            _world=world,
        )
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

        world.add_view(robot, exists_ok=True)

        return robot
